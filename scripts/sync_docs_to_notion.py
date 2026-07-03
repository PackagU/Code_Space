#!/usr/bin/env python3
"""
docs/ -> Notion wiki sync (one-way, idempotent).

Source of truth is the local docs/ tree.  Notion is a read-only view --
running this script overwrites the corresponding Notion page contents.

Usage:
    python scripts/sync_docs_to_notion.py [--dry-run] [--verbose]

Required env in .env (see .env.example):
    NOTION_TOKEN           Notion internal integration secret
    NOTION_PARENT_PAGE_ID  Parent page (the script creates a wiki root under this)

Manifest: docs/.notion_sync.json -- maps docs-relative paths to Notion page IDs.
File rename/move: edit the manifest key by hand (preserves the page ID, history).
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

try:
    import requests
except ImportError:
    sys.stderr.write("requests not installed.  Run: pip install -r scripts/requirements_notion_sync.txt\n")
    sys.exit(2)

try:
    from markdown_it import MarkdownIt
    from markdown_it.tree import SyntaxTreeNode
except ImportError:
    sys.stderr.write("markdown-it-py not installed.  Run: pip install -r scripts/requirements_notion_sync.txt\n")
    sys.exit(2)

# === Config ====================================================================

REPO_ROOT = Path(__file__).resolve().parent.parent
DOCS_DIR = REPO_ROOT / "docs"
MANIFEST_PATH = DOCS_DIR / ".notion_sync.json"
ENV_PATH = REPO_ROOT / ".env"

NOTION_API_BASE = "https://api.notion.com/v1"
NOTION_VERSION = "2022-06-28"
REQUEST_TIMEOUT = 30
RATE_LIMIT_SLEEP = 0.34  # ~3 req/sec safety margin

WIKI_ROOT_TITLE = "\U0001F4DA 문서 위키"  # "📚 문서 위키"
WIKI_ROOT_INTRO = (
    "docs/ 폴더를 자동 동기화한 위키. "
    "수정은 항상 로컬 docs/ 에서. "
    "Notion에서 직접 편집한 내용은 다음 sync에 덮어쓰여짐. "
    "동기화: python scripts/sync_docs_to_notion.py"
)

EXCLUDED_NAMES = {".notion_sync.json", "_style"}
EXCLUDED_PREFIXES = (".", "_")
EXCLUDED_SUFFIXES = (".tmp", ".swp", ".swo", ".bak")

GITHUB_RAW_URL_BASE = "https://raw.githubusercontent.com/PackagU/ros2-humble-slam-docker/main"

NOTION_CODE_LANGS = {
    "abap", "arduino", "bash", "basic", "c", "clojure", "coffeescript", "c++", "c#", "css",
    "dart", "diff", "docker", "elixir", "elm", "erlang", "flow", "fortran", "f#", "gherkin",
    "glsl", "go", "graphql", "groovy", "haskell", "html", "java", "javascript", "json",
    "julia", "kotlin", "latex", "less", "lisp", "livescript", "lua", "makefile", "markdown",
    "markup", "matlab", "mermaid", "nix", "objective-c", "ocaml", "pascal", "perl", "php",
    "plain text", "powershell", "prolog", "protobuf", "python", "r", "reason", "ruby",
    "rust", "sass", "scala", "scheme", "scss", "shell", "sql", "swift", "toml", "typescript",
    "vb.net", "verilog", "vhdl", "visual basic", "webassembly", "xml", "yaml",
}
LANG_ALIASES = {
    "sh": "shell", "bash": "bash", "zsh": "shell",
    "js": "javascript", "ts": "typescript",
    "yml": "yaml", "txt": "plain text", "text": "plain text",
    "dockerfile": "docker", "cpp": "c++", "csharp": "c#", "fsharp": "f#",
    "py": "python", "rb": "ruby", "rs": "rust", "kt": "kotlin",
}

NOTION_RICH_TEXT_LIMIT = 2000
NOTION_CHILDREN_PER_REQUEST = 100

# === Logging ===================================================================

_verbose = False

def log(msg: str, *, level: str = "info") -> None:
    prefix = {"info": "*", "ok": "+", "warn": "!", "err": "x", "dry": ">"}[level]
    print(f"{prefix} {msg}", flush=True)

def vlog(msg: str) -> None:
    if _verbose:
        log(msg, level="info")

# === .env loading (minimal, no python-dotenv dep) =============================

def load_env(env_path: Path) -> dict[str, str]:
    if not env_path.exists():
        log(f".env not found at {env_path}.  Copy .env.example to .env first.", level="err")
        sys.exit(2)
    out: dict[str, str] = {}
    for line in env_path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        out[k.strip()] = v.strip().strip('"').strip("'")
    return out

# === Manifest ==================================================================

@dataclass
class Manifest:
    root_page_id: str | None = None
    repo_url: str = "https://github.com/PackagU/ros2-humble-slam-docker"
    repo_branch: str = "main"
    paths: dict[str, str] = field(default_factory=dict)

    @classmethod
    def load(cls) -> "Manifest":
        if not MANIFEST_PATH.exists():
            return cls()
        raw = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        return cls(
            root_page_id=raw.get("_root_page_id"),
            repo_url=raw.get("_repo_url", cls.repo_url),
            repo_branch=raw.get("_repo_branch", cls.repo_branch),
            paths=raw.get("paths", {}),
        )

    def save(self) -> None:
        out = {
            "_comment": "Auto-managed by scripts/sync_docs_to_notion.py.  On rename/move, edit only the 'paths' key (preserves the Notion page ID).",
            "_root_page_id": self.root_page_id,
            "_repo_url": self.repo_url,
            "_repo_branch": self.repo_branch,
            "paths": dict(sorted(self.paths.items())),
        }
        MANIFEST_PATH.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

# === Notion API client =========================================================

class NotionClient:
    def __init__(self, token: str, *, dry_run: bool = False) -> None:
        self.token = token
        self.dry_run = dry_run
        self.session = requests.Session()
        self.session.headers.update({
            "Authorization": f"Bearer {token}",
            "Notion-Version": NOTION_VERSION,
            "Content-Type": "application/json",
        })

    def _request(self, method: str, path: str, json_body: dict | None = None) -> dict:
        url = f"{NOTION_API_BASE}{path}"
        last_err = None
        for attempt in range(4):
            try:
                r = self.session.request(method, url, json=json_body, timeout=REQUEST_TIMEOUT)
            except requests.RequestException as e:
                last_err = e
                wait = 2 ** attempt
                log(f"network error: {e}; retry in {wait}s", level="warn")
                time.sleep(wait)
                continue
            if r.status_code == 429:
                wait = float(r.headers.get("Retry-After", "1"))
                log(f"429 rate-limited; sleeping {wait}s", level="warn")
                time.sleep(wait)
                continue
            if 500 <= r.status_code < 600:
                wait = 2 ** attempt
                log(f"{r.status_code} server error; retry in {wait}s", level="warn")
                time.sleep(wait)
                continue
            if not r.ok:
                raise RuntimeError(f"Notion API {method} {path} -> {r.status_code}: {r.text[:500]}")
            time.sleep(RATE_LIMIT_SLEEP)
            return r.json()
        raise RuntimeError(f"Notion API {method} {path} failed after retries (last={last_err})")

    def create_page(self, parent_page_id: str, title: str, children: list[dict]) -> str:
        if self.dry_run:
            log(f"[DRY] create page '{title}' under {parent_page_id[:8]}", level="dry")
            return f"DRY-{abs(hash(title)) % 10**12:012d}"
        first_batch = children[:NOTION_CHILDREN_PER_REQUEST]
        rest = children[NOTION_CHILDREN_PER_REQUEST:]
        body: dict[str, Any] = {
            "parent": {"page_id": parent_page_id},
            "properties": {"title": {"title": [{"type": "text", "text": {"content": title[:200]}}]}},
        }
        if first_batch:
            body["children"] = first_batch
        resp = self._request("POST", "/pages", body)
        page_id = resp["id"]
        if rest:
            self.append_blocks(page_id, rest)
        return page_id

    def update_page_title(self, page_id: str, title: str) -> None:
        if self.dry_run:
            log(f"[DRY] update title of {page_id[:8]} -> '{title}'", level="dry")
            return
        body = {"properties": {"title": {"title": [{"type": "text", "text": {"content": title[:200]}}]}}}
        self._request("PATCH", f"/pages/{page_id}", body)

    def list_children(self, block_id: str) -> list[dict]:
        results: list[dict] = []
        cursor: str | None = None
        while True:
            qs = f"?start_cursor={cursor}" if cursor else ""
            resp = self._request("GET", f"/blocks/{block_id}/children{qs}")
            results.extend(resp.get("results", []))
            if not resp.get("has_more"):
                break
            cursor = resp.get("next_cursor")
        return results

    def delete_block(self, block_id: str) -> None:
        if self.dry_run:
            return
        self._request("DELETE", f"/blocks/{block_id}")

    def append_blocks(self, parent_block_id: str, blocks: list[dict]) -> None:
        if not blocks:
            return
        if self.dry_run:
            log(f"[DRY] append {len(blocks)} blocks to {parent_block_id[:8]}", level="dry")
            return
        for i in range(0, len(blocks), NOTION_CHILDREN_PER_REQUEST):
            chunk = blocks[i:i + NOTION_CHILDREN_PER_REQUEST]
            self._request("PATCH", f"/blocks/{parent_block_id}/children", {"children": chunk})

    def replace_page_content(self, page_id: str, blocks: list[dict]) -> None:
        existing = self.list_children(page_id)
        deletable = [b for b in existing if b.get("type") != "child_page"]
        if self.dry_run:
            log(f"[DRY] delete {len(deletable)} existing blocks (preserve {len(existing) - len(deletable)} sub-pages), append {len(blocks)} new", level="dry")
            return
        for blk in deletable:
            try:
                self.delete_block(blk["id"])
            except Exception as e:
                log(f"failed to delete block {blk['id'][:8]}: {e}", level="warn")
        self.append_blocks(page_id, blocks)

# === Markdown -> Notion blocks ================================================

def _chunk(s: str, limit: int = NOTION_RICH_TEXT_LIMIT) -> list[str]:
    return [s[i:i + limit] for i in range(0, len(s), limit)] or [""]

def rt(text: str, *, bold: bool = False, italic: bool = False,
       strikethrough: bool = False, code: bool = False,
       link: str | None = None) -> list[dict]:
    items: list[dict] = []
    for chunk in _chunk(text):
        item: dict[str, Any] = {
            "type": "text",
            "text": {"content": chunk, "link": ({"url": link} if link else None)},
            "annotations": {
                "bold": bold, "italic": italic, "strikethrough": strikethrough,
                "underline": False, "code": code, "color": "default",
            },
        }
        items.append(item)
    return items

def normalize_lang(lang: str) -> str:
    if not lang:
        return "plain text"
    lang = lang.lower().strip().split()[0] if lang.strip() else "plain text"
    lang = LANG_ALIASES.get(lang, lang)
    if lang in NOTION_CODE_LANGS:
        return lang
    return "plain text"


class MarkdownToNotion:
    def __init__(self, *, doc_path: Path, repo_branch: str, repo_url: str) -> None:
        self.doc_path = doc_path
        self.repo_branch = repo_branch
        self.repo_url = repo_url
        self.md = (MarkdownIt("commonmark", {"html": False, "linkify": False, "typographer": False})
                   .enable("table").enable("strikethrough"))

    def transform_image_url(self, src: str) -> str:
        if src.startswith(("http://", "https://", "data:")):
            return src
        abs_target = (self.doc_path.parent / src).resolve()
        try:
            rel_from_repo = abs_target.relative_to(REPO_ROOT)
        except ValueError:
            log(f"image {src} resolves outside repo; using as-is", level="warn")
            return src
        return f"{GITHUB_RAW_URL_BASE}/{rel_from_repo.as_posix()}"

    def parse(self, source: str) -> tuple[str | None, list[dict]]:
        title, body = self._strip_frontmatter(source)
        tokens = self.md.parse(body)
        tree = SyntaxTreeNode(tokens)
        blocks: list[dict] = []
        first_h1_consumed = False
        for node in tree.children:
            if (not first_h1_consumed and node.type == "heading" and node.tag == "h1"):
                first_h1_consumed = True
                if not title:
                    title = self._inline_to_plaintext(node)
                continue
            self._convert_node(node, blocks)
        return title, blocks

    @staticmethod
    def _strip_frontmatter(source: str) -> tuple[str | None, str]:
        if not source.startswith("---\n"):
            return None, source
        end = source.find("\n---\n", 4)
        if end == -1:
            return None, source
        fm_text = source[4:end]
        body = source[end + 5:]
        title: str | None = None
        for line in fm_text.splitlines():
            if line.strip().startswith("title:"):
                title = line.split(":", 1)[1].strip().strip('"').strip("'")
                break
        return title, body

    def _convert_node(self, node: SyntaxTreeNode, out: list[dict]) -> None:
        t = node.type
        if t == "heading":
            level = min(int(node.tag[1]), 3)
            key = f"heading_{level}"
            out.append({"object": "block", "type": key, key: {"rich_text": self._inline_richtext(node)}})
        elif t == "paragraph":
            img = self._extract_solo_image(node)
            if img:
                out.append(img)
                return
            out.append({"object": "block", "type": "paragraph",
                        "paragraph": {"rich_text": self._inline_richtext(node)}})
        elif t == "fence" or t == "code_block":
            lang = normalize_lang(getattr(node, "info", "") or "")
            content = node.content or ""
            out.append({"object": "block", "type": "code",
                        "code": {"rich_text": rt(content), "language": lang}})
        elif t == "bullet_list":
            for li in node.children:
                self._convert_list_item(li, "bulleted_list_item", out)
        elif t == "ordered_list":
            for li in node.children:
                self._convert_list_item(li, "numbered_list_item", out)
        elif t == "blockquote":
            for child in node.children:
                if child.type == "paragraph":
                    out.append({"object": "block", "type": "quote",
                                "quote": {"rich_text": self._inline_richtext(child)}})
                else:
                    self._convert_node(child, out)
        elif t == "hr":
            out.append({"object": "block", "type": "divider", "divider": {}})
        elif t == "table":
            self._convert_table(node, out)
        elif t in ("html_block", "html_inline"):
            preview = (node.content or "")[:60].replace("\n", " ")
            log(f"  HTML skipped in {self.doc_path.name}: {preview!r}", level="warn")
        else:
            text = self._inline_to_plaintext(node)
            if text:
                out.append({"object": "block", "type": "paragraph",
                            "paragraph": {"rich_text": rt(text)}})

    def _convert_list_item(self, li: SyntaxTreeNode, block_type: str, out: list[dict]) -> None:
        rich_text_parts: list[dict] = []
        nested: list[dict] = []
        is_todo = False
        todo_checked = False
        for idx, child in enumerate(li.children):
            if child.type == "paragraph":
                pt = self._inline_to_plaintext(child)
                if idx == 0 and pt[:4] in ("[ ] ", "[x] ", "[X] "):
                    is_todo = True
                    todo_checked = pt[1] in ("x", "X")
                    rt_items = self._inline_richtext(child)
                    if rt_items and rt_items[0]["text"]["content"][:4] in ("[ ] ", "[x] ", "[X] "):
                        rt_items[0]["text"]["content"] = rt_items[0]["text"]["content"][4:]
                    rich_text_parts.extend(rt_items)
                elif idx == 0:
                    rich_text_parts.extend(self._inline_richtext(child))
                else:
                    nested.append({"object": "block", "type": "paragraph",
                                   "paragraph": {"rich_text": self._inline_richtext(child)}})
            elif child.type in ("bullet_list", "ordered_list"):
                nt = "bulleted_list_item" if child.type == "bullet_list" else "numbered_list_item"
                for nested_li in child.children:
                    self._convert_list_item(nested_li, nt, nested)
            else:
                self._convert_node(child, nested)

        final_type = "to_do" if is_todo else block_type
        block: dict[str, Any] = {"object": "block", "type": final_type,
                                 final_type: {"rich_text": rich_text_parts}}
        if is_todo:
            block[final_type]["checked"] = todo_checked
        if nested:
            block[final_type]["children"] = nested
        out.append(block)

    def _convert_table(self, node: SyntaxTreeNode, out: list[dict]) -> None:
        rows: list[list[list[dict]]] = []
        has_header = False
        for section in node.children:
            section_is_head = (section.type == "thead")
            for tr in section.children:
                row_cells = [self._inline_richtext(cell) for cell in tr.children]
                rows.append(row_cells)
            if section_is_head and rows:
                has_header = True
        if not rows:
            return
        width = max(len(r) for r in rows)
        for r in rows:
            while len(r) < width:
                r.append([])
        out.append({
            "object": "block", "type": "table",
            "table": {
                "table_width": width,
                "has_column_header": has_header,
                "has_row_header": False,
                "children": [
                    {"object": "block", "type": "table_row", "table_row": {"cells": row}}
                    for row in rows
                ],
            },
        })

    def _extract_solo_image(self, paragraph: SyntaxTreeNode) -> dict | None:
        if not paragraph.children:
            return None
        inline = paragraph.children[0]
        if inline.type != "inline":
            return None
        meaningful = [c for c in inline.children if not (c.type == "text" and not (c.content or "").strip())]
        if len(meaningful) == 1 and meaningful[0].type == "image":
            img = meaningful[0]
            src = img.attrs.get("src", "")
            alt = img.attrs.get("alt", "") or img.content or ""
            url = self.transform_image_url(src)
            return {"object": "block", "type": "image",
                    "image": {"type": "external", "external": {"url": url},
                              "caption": rt(alt) if alt else []}}
        return None

    def _inline_richtext(self, node: SyntaxTreeNode) -> list[dict]:
        container = node
        if node.children and len(node.children) == 1 and node.children[0].type == "inline":
            container = node.children[0]
        items: list[dict] = []
        self._walk_inline(container, items,
                          ann={"bold": False, "italic": False, "strikethrough": False, "code": False},
                          link=None)
        return items

    def _walk_inline(self, node: SyntaxTreeNode, out: list[dict], *,
                     ann: dict, link: str | None) -> None:
        for child in node.children:
            ct = child.type
            if ct == "text":
                if child.content:
                    out.extend(rt(child.content, bold=ann["bold"], italic=ann["italic"],
                                  strikethrough=ann["strikethrough"], code=ann["code"], link=link))
            elif ct in ("softbreak", "hardbreak"):
                out.extend(rt("\n", bold=ann["bold"], italic=ann["italic"],
                              strikethrough=ann["strikethrough"], code=ann["code"], link=link))
            elif ct == "strong":
                self._walk_inline(child, out, ann={**ann, "bold": True}, link=link)
            elif ct == "em":
                self._walk_inline(child, out, ann={**ann, "italic": True}, link=link)
            elif ct == "s":
                self._walk_inline(child, out, ann={**ann, "strikethrough": True}, link=link)
            elif ct == "code_inline":
                if child.content:
                    out.extend(rt(child.content, bold=ann["bold"], italic=ann["italic"],
                                  strikethrough=ann["strikethrough"], code=True, link=link))
            elif ct == "link":
                href = child.attrs.get("href", "") or ""
                if not (href.startswith("http://") or href.startswith("https://") or href.startswith("mailto:")):
                    href = None
                self._walk_inline(child, out, ann=ann, link=href)
            elif ct == "image":
                alt = child.attrs.get("alt", "") or child.content or ""
                if alt:
                    out.extend(rt(f"[{alt}]", bold=ann["bold"], italic=ann["italic"],
                                  strikethrough=ann["strikethrough"], code=False, link=link))
            elif ct == "html_inline":
                pass
            else:
                if child.children:
                    self._walk_inline(child, out, ann=ann, link=link)
                elif child.content:
                    out.extend(rt(child.content, bold=ann["bold"], italic=ann["italic"],
                                  strikethrough=ann["strikethrough"], code=ann["code"], link=link))

    def _inline_to_plaintext(self, node: SyntaxTreeNode) -> str:
        items = self._inline_richtext(node)
        return "".join(it["text"]["content"] for it in items)

# === File walker ===============================================================

def is_excluded(name: str) -> bool:
    if name in EXCLUDED_NAMES:
        return True
    if name.startswith(EXCLUDED_PREFIXES):
        return True
    if name.endswith(EXCLUDED_SUFFIXES):
        return True
    return False

@dataclass
class TreeItem:
    rel_path: str
    abs_path: Path
    is_folder: bool
    parent_rel: str | None

def walk_docs() -> list[TreeItem]:
    items: list[TreeItem] = []
    for current_dir, dirnames, filenames in os.walk(DOCS_DIR):
        dirnames[:] = sorted(d for d in dirnames if not is_excluded(d))
        filenames = sorted(f for f in filenames if not is_excluded(f))

        cur_path = Path(current_dir)
        rel_cur = cur_path.relative_to(DOCS_DIR).as_posix()

        if rel_cur != ".":
            parent_obj = Path(rel_cur).parent.as_posix()
            parent = parent_obj if parent_obj != "." else None
            items.append(TreeItem(rel_path=rel_cur, abs_path=cur_path, is_folder=True, parent_rel=parent))

        for fname in filenames:
            if not fname.endswith(".md"):
                continue
            if fname == "README.md":
                continue
            rel_file = (Path(rel_cur) / fname).as_posix() if rel_cur != "." else fname
            parent = rel_cur if rel_cur != "." else None
            items.append(TreeItem(rel_path=rel_file, abs_path=cur_path / fname, is_folder=False, parent_rel=parent))
    return items

def derive_title_from_name(rel_path: str, is_folder: bool) -> str:
    name = Path(rel_path).name if is_folder else Path(rel_path).stem
    m = re.match(r"^\d+[_-](.+)$", name)
    if m:
        name = m.group(1)
    return name.replace("_", " ")

# === Sync orchestration ========================================================

def ensure_wiki_root(client: NotionClient, manifest: Manifest, parent_page_id: str) -> str:
    if manifest.root_page_id and not manifest.root_page_id.startswith("DRY-"):
        vlog(f"wiki root exists: {manifest.root_page_id}")
        return manifest.root_page_id
    log(f"creating wiki root '{WIKI_ROOT_TITLE}' under {parent_page_id[:8]}...")
    intro = [{
        "object": "block", "type": "callout",
        "callout": {"rich_text": rt(WIKI_ROOT_INTRO),
                    "icon": {"type": "emoji", "emoji": "\U0001F4CC"},
                    "color": "gray_background"},
    }]
    page_id = client.create_page(parent_page_id, WIKI_ROOT_TITLE, intro)
    manifest.root_page_id = page_id
    return page_id

def sync_item(client: NotionClient, manifest: Manifest, item: TreeItem, parent_page_id: str) -> None:
    existing = manifest.paths.get(item.rel_path)

    if item.is_folder:
        readme = item.abs_path / "README.md"
        if readme.exists():
            converter = MarkdownToNotion(doc_path=readme, repo_branch=manifest.repo_branch, repo_url=manifest.repo_url)
            title_override, blocks = converter.parse(readme.read_text(encoding="utf-8"))
            title = title_override or derive_title_from_name(item.rel_path, True)
        else:
            title = derive_title_from_name(item.rel_path, True)
            blocks = [{
                "object": "block", "type": "callout",
                "callout": {"rich_text": rt(f"{item.rel_path}/ -- 하위 페이지를 참고하세요."),
                            "icon": {"type": "emoji", "emoji": "\U0001F4C1"}, "color": "default"},
            }]
    else:
        converter = MarkdownToNotion(doc_path=item.abs_path, repo_branch=manifest.repo_branch, repo_url=manifest.repo_url)
        title_override, blocks = converter.parse(item.abs_path.read_text(encoding="utf-8"))
        title = title_override or derive_title_from_name(item.rel_path, False)

    if existing and not existing.startswith("DRY-"):
        log(f"update  {item.rel_path}  ->  {existing[:8]}")
        client.update_page_title(existing, title)
        client.replace_page_content(existing, blocks)
    else:
        log(f"create  {item.rel_path}  (under {parent_page_id[:8]})")
        page_id = client.create_page(parent_page_id, title, blocks)
        manifest.paths[item.rel_path] = page_id

def sync_all(client: NotionClient, manifest: Manifest, parent_page_id: str) -> None:
    wiki_root = ensure_wiki_root(client, manifest, parent_page_id)
    items = walk_docs()
    log(f"docs/ items: {sum(1 for i in items if i.is_folder)} folders, {sum(1 for i in items if not i.is_folder)} files")
    for item in items:
        parent_id = wiki_root if item.parent_rel is None else manifest.paths.get(item.parent_rel)
        if parent_id is None:
            log(f"missing parent for {item.rel_path} (parent={item.parent_rel}) -- skipping", level="err")
            continue
        try:
            sync_item(client, manifest, item, parent_id)
        except Exception as e:
            log(f"failed on {item.rel_path}: {e}", level="err")

    fs_paths = {it.rel_path for it in items}
    stale = [p for p in manifest.paths if p not in fs_paths]
    if stale:
        log(f"{len(stale)} stale manifest entries (no longer in docs/):", level="warn")
        for p in stale:
            log(f"   {p}  ->  {manifest.paths[p][:8]}", level="warn")
        log("   (kept; remove from manifest manually if you want them gone from Notion)", level="warn")

# === CLI =======================================================================

def main() -> int:
    global _verbose
    p = argparse.ArgumentParser(description="Sync docs/ -> Notion wiki")
    p.add_argument("--dry-run", action="store_true", help="print actions without calling Notion API")
    p.add_argument("--verbose", "-v", action="store_true", help="extra logging")
    args = p.parse_args()
    _verbose = args.verbose

    env = load_env(ENV_PATH)
    token = env.get("NOTION_TOKEN", "")
    parent_page_id = env.get("NOTION_PARENT_PAGE_ID", "")
    if not token or "PLACEHOLDER" in token:
        log("NOTION_TOKEN not set in .env -- see .env.example", level="err")
        return 2
    if not parent_page_id:
        log("NOTION_PARENT_PAGE_ID not set in .env", level="err")
        return 2

    manifest = Manifest.load()
    client = NotionClient(token, dry_run=args.dry_run)

    try:
        sync_all(client, manifest, parent_page_id)
    finally:
        if not args.dry_run:
            manifest.save()
            log(f"manifest saved: docs/.notion_sync.json", level="ok")
        else:
            log("dry-run: manifest NOT saved", level="dry")

    log("done", level="ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())

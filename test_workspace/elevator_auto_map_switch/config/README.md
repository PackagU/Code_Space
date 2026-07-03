# Config

작성일: 2026-06-02 · 갱신: 2026-06-12 (구현 완료)

## 현재 파일

| 파일 | 역할 |
|------|------|
| `floor_maps.yaml` | 층별 map yaml 경로 + initialpose용 포즈(elevator_inside/exit). 경로는 프로젝트 루트 기준 상대경로로 적고, resolve 규칙은 파일 상단 주석과 `FeedBack/02_floor_maps_yaml.md` 참조 |

계획 단계에서 후보였던 `auto_map_switch.yaml`(정책/서비스명/타임아웃)과 `spawn_poses.yaml`은 별도 파일 대신 노드 파라미터와 `floor_maps.yaml`의 `points`로 흡수했다 — 파일 수를 늘릴 만큼 내용이 많지 않았다.

## 주의

- `test_workspace/elevator_mission/`의 설정은 legacy 기준으로 둔다.
- 민감정보나 외부 서비스 credential은 절대 넣지 않는다.

# 업로드 전 검사

판정: **오프라인 검증**. 2026-09-16 현재 회수본에 적용한 검사다.

- Python AST 구문: [측정값] 144개 통과
- YAML·XML·JSON 파싱: [측정값] 34·16·8개 통과
- F1·F2·F3 현재 지도 YAML·PGM과 pin SHA256 일치
- PASS portability check (209 files scanned)
- Jetson `test_field_scripts_contract.py` 통과: 셸 구문·정적 계약 검사
- 허용 파일 비밀 패턴·대용량 검사 통과
- 소스·문서 `git diff --cached --check` 통과. `evidence/` 원문 로그의 기존 줄 끝 공백은 보존하고 공백 검사에서 제외

이번 검사는 로봇·팔·리프트를 구동하지 않았다. 실주행 결과는 [주행 결과표](./README.md)와 별도 근거다.

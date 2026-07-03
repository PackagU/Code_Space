# 요약

<!-- 무엇을, 왜 바꿨는지 2~3줄 -->

## 연결

- Refs: `Roadmap/<파일>.md` 또는 `Closes #<이슈번호>`
- improvement_report 갱신 필요 여부: 예 / 아니오

## 체크리스트

- [ ] `bash scripts/run_offline_tests.sh` 통과 (또는 CI green)
- [ ] `python3 scripts/check_portability.py` 통과
- [ ] 시뮬 동작 변경 시: 컨테이너에서 `run_l3_world_swap_smoke.sh` PASS
- [ ] 민감정보(.env/token/개인정보) 미포함 확인 (AGENTS.md 규칙 7~9)
- [ ] 대용량 바이너리(webm 등) 미포함

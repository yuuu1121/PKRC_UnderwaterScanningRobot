# Claude Code 하네스 명령어 총정리

> 설치 기반: [luckkim123/claudebase](https://github.com/luckkim123/claudebase) (`~/claudebase` → `~/.claude` 심링크)
> 플러그인 명령은 `/플러그인이름:명령` 형태로도 호출 가능 (예: `/oh-my-claudecode:debug`)

---

## 1. claudebase 기본 스킬 (5개)

| 명령 | 설명 |
|:---|:---|
| `/changelog` | 이번 세션의 결정·실험·교훈을 레포 루트 `changelog.md`에 기록하고 커밋. "오늘 한 거 정리"라고 말해도 발동 |
| `/gen-image <출력폴더> <프롬프트>` | Google nano banana로 이미지 1장 생성 후 PNG 저장. "그려줘", "썸네일 만들어"로도 발동. ⚠️ gemini CLI 필요(미설치) |
| `/memory-update` | 자동 메모리(`~/.claude/projects/<프로젝트>/memory/`) 압축·정리. "메모리 정리"로도 발동 |
| `/readme-project <경로>` | 프로젝트 코드베이스를 분석해 OSS 수준 README.md 생성 |
| `/sync-claudebase` | claudebase 레포 머신 간 동기화 점검 (fetch → diff → 드리프트 체크 → install → 검증) |

---

## 2. OMC (oh-my-claudecode) — 멀티에이전트 오케스트레이션

### 매직 키워드 (프롬프트에 그냥 쓰면 자동 발동)

| 키워드 | 설명 |
|:---|:---|
| `autopilot: <목표>` | 목표만 주면 계획→구현→검증까지 완전 자율 실행 |
| `ralph <작업>` | 완료될 때까지 자가 반복 루프 (실패해도 계속 재시도) |
| `ultrawork <작업>` | 병렬 에이전트를 최대로 동원해 작업 수행 |
| `team <작업>` | 에이전트 팀 구성해서 협업 수행 |
| `cancel` | 실행 중인 autopilot/ralph 등 중단 |

### 주요 스킬/명령

| 명령 | 설명 |
|:---|:---|
| `/oh-my-claudecode:plan` | 구현 계획 수립 (인터뷰 방식) |
| `ralplan` | ralph 루프용 계획 수립 |
| `/oh-my-claudecode:ask` | 코드 수정 없이 질문·분석만 |
| `/oh-my-claudecode:debug` | 체계적 디버깅 세션 (가설→검증 루프) |
| `/oh-my-claudecode:deep-dive` | 코드베이스 심층 분석 |
| `deep-interview` | 요구사항 심층 인터뷰 |
| `/oh-my-claudecode:deepinit` | 프로젝트 CLAUDE.md 심층 생성 (코드 전체 분석) |
| `/oh-my-claudecode:autoresearch` | 자율 리서치 (웹+코드) |
| `/oh-my-claudecode:sciomc` | 과학/실험 워크플로 오케스트레이션 |
| `ultragoal` | 대형 목표를 하위 목표로 분해해 순차 달성 |
| `ultraqa` | 철저한 QA/테스트 수행 |
| `verify` | 변경사항 실제 동작 검증 |
| `trace` | 코드 실행 경로 추적 |
| `wiki` | 코드베이스 위키 생성/조회 |
| `skillify` / `skill` | 반복 작업을 스킬로 성문화 |
| `learner` / `self-improve` | 세션에서 배운 것 학습·자기개선 |
| `/oh-my-claudecode:remember` | 학습 내용 메모리 저장 |
| `ai-slop-cleaner` | AI 특유의 저품질 코드/문장 정리 |
| `writer-memory` / `writer` | 글쓰기 맥락 유지 |
| `/oh-my-claudecode:release` | 버전 릴리스 워크플로 |
| `/oh-my-claudecode:compact` | 컨텍스트 압축 |
| `/oh-my-claudecode:hud` | statusLine HUD 설정 (`hud setup`, preset 변경) |
| `/oh-my-claudecode:omc-doctor` | OMC 설치 상태 진단 |
| `/oh-my-claudecode:omc-setup` | OMC 초기 설정 |
| `/oh-my-claudecode:omc-teams` | 에이전트 팀 구성 관리 |
| `/oh-my-claudecode:psm` | 프로젝트 세션 관리자 (여러 세션 이어가기) |
| `/oh-my-claudecode:mcp-setup` | MCP 서버 설정 도우미 |
| `/oh-my-claudecode:configure-notifications` | 알림 설정 |
| `/oh-my-claudecode:external-context` | 외부 문서/컨텍스트 주입 |
| `/oh-my-claudecode:ccg` | Claude Code 가이드 질문 |

### OMC 에이전트 (19개 — 오케스트레이션 시 자동 선택되며, 직접 지정도 가능)

| 에이전트 | 역할 |
|:---|:---|
| `planner` | 계획 수립 |
| `architect` | 아키텍처 설계 |
| `executor` | 코드 구현 실행 |
| `explore` | 코드베이스 탐색 |
| `analyst` | 분석 |
| `debugger` | 디버깅 |
| `tracer` | 실행 경로 추적 |
| `code-reviewer` | 코드 리뷰 |
| `code-simplifier` | 코드 단순화 |
| `security-reviewer` | 보안 리뷰 |
| `test-engineer` | 테스트 작성 |
| `qa-tester` | QA 테스트 |
| `verifier` | 동작 검증 |
| `critic` | 비판적 검토 |
| `designer` | UI/UX 설계 |
| `scientist` | 실험/과학 분석 |
| `document-specialist` | 문헌/자료 리서치 |
| `writer` | 문서 작성 |
| `git-master` | git 작업 |

---

## 3. oh-my-docs (omd) — 문서 생성 파이프라인

PPT / DOCX / HWPX 문서 생성. `docs-pilot`이 전체 자동 조율.
⚠️ Python venv 필요 — 설치됨 (`~/.claude/.venv`, settings.local.json에 PATH 등록됨)

| 명령 | 설명 |
|:---|:---|
| `docs-pilot` | 문서 생성 전 과정 자동 조율 (아래 단계들을 알아서 실행) |
| `docs-intake` | 요구사항 수집 (어떤 문서? 누구에게? 무슨 내용?) |
| `docs-plan` | 문서 구성 설계 (목차, 슬라이드 구성) |
| `docs-build` | 실제 문서 빌드 (pptx/docx/hwpx 생성) |
| `docs-verify` | 생성된 문서 검증 |
| `docs-revise` | 피드백 반영 수정 |
| `docs-convert` | 문서 형식 변환 |
| `docs-translate` | 문서 번역 |
| `docs-inspect` | 기존 문서 구조 분석 |
| `docs-standardize` | 문서 표준화 (서식 통일) |
| `docs-learn` | 문서 스타일 학습 |

---

## 4. oh-my-project (omp) — 프로젝트 관리

| 명령 | 설명 |
|:---|:---|
| `omp-init` | 프로젝트 구조 초기화 |
| `omp-audit` | 프로젝트 상태 감사 |
| `omp-organize` | 파일/폴더 정리 |
| `omp-doc` | 프로젝트 문서화 |
| `omp-dataset` | 데이터셋 관리 |
| `omp-env` | 환경 설정 관리 |
| `omp-doctor` | 프로젝트 상태 진단 |
| `omp-learn` | 프로젝트에서 배운 것 학습 |
| `omp-codify` | 학습 내용을 규칙으로 성문화 |
| `omp-pilot` | 프로젝트 관리 전 과정 자동 조율 |

---

## 5. oh-my-experiments (omx) — 실험 관리

ROV 게인 튜닝처럼 반복 실험에 적합한 구조.

| 명령 | 설명 |
|:---|:---|
| `exp-init` | 실험 프로젝트 초기화 |
| `exp-design` | 실험 설계 (변수, 조건, 측정 항목) |
| `exp-loop` | 실험 실행 루프 (실행→기록→다음 조건) |
| `exp-analyze` | 실험 결과 분석 |

---

## 6. superpowers — 검증된 개발 절차 모음

작업 상황에 맞으면 자동 발동. 명시적으로 부를 수도 있음.

| 스킬 | 설명 |
|:---|:---|
| `brainstorming` | 구현 전 아이디어 발산·수렴 |
| `writing-plans` | 구현 계획서 작성 |
| `executing-plans` | 계획서 기반 단계 실행 |
| `test-driven-development` | TDD (테스트 먼저) |
| `systematic-debugging` | 체계적 디버깅 절차 |
| `verification-before-completion` | 완료 선언 전 검증 |
| `requesting-code-review` / `receiving-code-review` | 코드 리뷰 요청/반영 |
| `dispatching-parallel-agents` | 병렬 에이전트 분배 |
| `subagent-driven-development` | 서브에이전트 주도 개발 |
| `using-git-worktrees` | git worktree 활용 |
| `finishing-a-development-branch` | 브랜치 마무리 (머지/정리) |
| `writing-skills` | 새 스킬 작성법 |

---

## 7. ponytail — 과잉 설계 방지 (게으른 최적해)

| 명령 | 설명 |
|:---|:---|
| `/ponytail` | 가장 단순한 해법 강제 모드. "lazy mode", "yagni"로도 발동 |
| `/ponytail-review` | diff에서 과잉 설계만 리뷰 (지울 것 찾기) |
| `/ponytail-audit` | 레포 전체 과잉 설계 감사 |
| `/ponytail-debt` | ponytail이 남긴 의도적 지름길(부채) 목록 |
| `/ponytail-gain` | ponytail 절감 효과 스코어보드 |
| `/ponytail-help` | ponytail 모드 도움말 |

---

## 8. 기타 플러그인

| 플러그인 | 사용법 |
|:---|:---|
| `context7` | 라이브러리 최신 공식 문서 자동 조회 (MCP, 자동 발동) |
| `pyright-lsp` / `clangd-lsp` | Python / C++ 코드 인텔리전스 (자동) |
| `axlabs-mckinsey-pptx` | `mckinsey-deck` — 맥킨지 스타일 PPT 덱 생성 |
| `claude-md-management` | `claude-md-improver`, `/revise-claude-md` — CLAUDE.md 개선 |
| `claude-code-setup` | `claude-automation-recommender` — 자동화 추천 |
| `learning-output-style` | 학습 모드 출력 스타일 |
| `oh-my-heroacademia` | heroacademia 마켓플레이스 허브 (스킬 카드 제공) |

미설치: `oh-my-scholar` (비공개 저장소 — `gh auth login` 후 `claude plugin install oh-my-scholar@heroacademia --scope user`)

---

## 9. 자동 훅 (호출 불필요 — 참고용)

| 훅 | 발동 시점 | 하는 일 |
|:---|:---|:---|
| askuserquestion-guard | 질문 툴 호출 전 | 빈/불완전한 질문 호출 차단 |
| askuserquestion_retry | 턴 종료 시 | 빈 질문 호출 감지 후 재시도 유도 |
| agent-routing-guard | 에이전트 호출 전 | 리서치 작업의 오라우팅 차단 |
| detect_malformed_toolcall | 턴 종료 시 | 텍스트로 샌 툴콜 마크업 감지 |
| fix_surrogate | 턴 종료 + 세션 시작 | 트랜스크립트 깨진 문자 자동 수리 (API 400 방지) |
| hud-ensure | 세션 시작 | HUD 커스터마이징 자동 복구 |
| OMC keyword-detector | 프롬프트 입력마다 | 매직 키워드(autopilot 등) 감지 |

---

## 10. tmux 명령어 (prefix: **Ctrl+a** — 기본 Ctrl+b 아님!)

### 세션 (터미널에서)

```bash
tmux new -s 이름        # 새 세션 시작
tmux attach -t 이름     # 세션 재접속
tmux ls                 # 세션 목록
```

### 키 바인딩 (Ctrl+a 누르고 나서)

| 키 | 동작 |
|:---|:---|
| `d` | 세션 분리(detach) — 작업은 백그라운드에서 계속 |
| `o` | 창 위/아래 분할 (현재 디렉토리 유지) |
| `e` | 창 좌/우 분할 (현재 디렉토리 유지) |
| `t` | 새 윈도우 |
| `w` | 창 닫기 (마지막 창이면 윈도우 닫기) |
| `[` | 이전 윈도우 |
| `]` | 다음 윈도우 |
| `n` | 윈도우 이름 변경 |
| `W` | 세션/윈도우 트리 뷰 |
| `s` | 모든 창 동시 입력 토글 (여러 장비 동시 명령) |
| `m` | 마우스 모드 on/off |
| `T` | 시계 표시 |
| `r` | tmux.conf 리로드 |
| `P` | 붙여넣기 |

### 마우스 복사/붙여넣기

| 동작 | 결과 |
|:---|:---|
| 드래그 | 선택 즉시 클립보드 복사 |
| 더블클릭 | 단어 복사 |
| 트리플클릭 | 줄 복사 |
| 가운데/우클릭 | 붙여넣기 |
| 휠 위로 | 스크롤 (복사 모드 자동 진입) |

- SSH 접속 중에도 OSC52로 로컬 PC 클립보드까지 복사됨
- VS Code 터미널에서 드래그가 멈추면: `Ctrl+a m`으로 마우스 끄고 네이티브 드래그 사용

### 복사 모드 (vi 스타일 — 휠 올리면 진입)

| 키 | 동작 |
|:---|:---|
| `v` | 선택 시작 |
| `y` 또는 `Enter` | 복사하고 종료 |
| `q` | 종료 |

---

## 11. 유지보수

```bash
cd ~/claudebase && git pull            # 설정 업데이트 (재설치 불필요)
~/claudebase/installer/install.sh      # mcp/플러그인 바뀐 경우만 재실행
```

- 설정 파일 위치: 공통 `~/claudebase/config/settings.json` (git), 머신 로컬 `~/.claude/settings.local.json`
- 설치 전 백업: `~/.claude/backups/pre-claudebase-20260708/`
- 문제 진단: `/oh-my-claudecode:omc-doctor`, `/sync-claudebase`

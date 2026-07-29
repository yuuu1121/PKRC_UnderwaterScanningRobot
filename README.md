# claudebase 하네스 사용법 정리

> [luckkim123/claudebase](https://github.com/luckkim123/claudebase) — 여러 머신에서 Claude Code 환경(설정·스킬·훅·플러그인)을 git + 심링크로 동기화하는 개인 rig.
> 로컬 클론 위치: `~/claudebase` (정본 경로 — 훅들이 이 절대경로를 참조하므로 바꾸면 안 됨)

## 1. 설치 / 동기화

```bash
git clone https://github.com/luckkim123/claudebase.git ~/claudebase
~/claudebase/installer/install.sh        # macOS / Linux
# pwsh installer/install.ps1             # Windows
```

`install.sh`가 하는 일:

| 대상 | 방식 |
|:---|:---|
| `~/.claude/settings.json` | → `config/settings.json` 심링크 (**기존 파일 백업 없이 교체됨 — 주의**) |
| `~/.claude/CLAUDE.md` | → `config/CLAUDE.md` 심링크 (행동 원칙, OMC 오케스트레이션 규칙) |
| `~/.claude/skills/<이름>` | → `runtime/skills/<이름>` 스킬별 개별 심링크 |
| `~/.claude/mcp.json` | `config/mcp.template.json` + `secrets/secrets.env`로 렌더링 |
| `~/.tmux.conf` | → `shell/tmux.conf` 심링크 |
| 플러그인 | `settings.json`의 `enabledPlugins` 기준으로 user-scope 동기화 (OMC, superpowers, oh-my-docs 등 14개) |

옵션: `--copy`(심링크 대신 복사), `--dry-run`(실행 없이 확인), `--verbose`, `--prune-plugins`(목록에 없는 플러그인 제거). `INSTALL_TOOLS=1`을 주면 tmux/클립보드 도구를 자동 설치.

업데이트는 `cd ~/claudebase && git pull`만 하면 됨(심링크라 재설치 불필요). mcp/플러그인이 바뀐 경우에만 `install.sh` 재실행.

**의존성**: `jq`(statusLine 필수), `gemini` CLI + nanobanana 확장(gen-image 스킬용), Python ≥3.10(문서 스킬 venv `~/.claude/.venv`). 없으면 경고만 출력되고 설치는 진행됨.

## 2. 스킬 (Slash 명령)

### `/changelog` — 세션 의사결정 기록
- **용도**: 이번 세션의 **결정·실험·교훈**을 레포 루트 `changelog.md`에 기록하고 세션에서 수정한 파일과 함께 커밋. 코드 diff가 아니라 git이 담지 못하는 "왜 그랬는지, 뭘 배웠는지"를 남김.
- **트리거**: `/changelog`, "체인지로그 정리", "이번 세션 기록", "오늘 한 거 정리"
- **쓰지 말 것**: 한 줄짜리 사소한 수정, 결정 내용 없는 단순 리팩터링 (커밋 노이즈만 늘어남)

### `/gen-image` — 이미지 생성 (nano banana)
- **용도**: Google nano banana(Gemini 2.5 Flash Image)로 이미지 1장을 생성해 PNG로 저장. 프롬프트를 Google의 5요소 장면 프롬프트로 다듬은 뒤 gemini CLI 경로 → REST 폴백으로 생성하고, **디스크에 파일이 실제 존재하는지 반드시 확인**.
- **사용법**: `/gen-image <출력디렉토리> <프롬프트> [--aspect 16:9|1:1|9:16] [--diagnose]`
- **트리거**: "그려줘", "이미지 만들어", "아이콘 생성", "포스터", "썸네일"
- **쓰지 말 것**: 기존 PNG 편집(별도 워크플로), 다이어그램/플로차트(mermaid 사용), 실존 인물 딥페이크(거부)

### `/memory-update` — 자동 메모리 압축
- **용도**: `~/.claude/projects/<프로젝트>/memory/`의 자동 메모리를 압축·정리 (MEMORY.md가 한도를 넘거나, 중복 항목이 쌓였을 때). 스키마는 시스템 프롬프트의 auto memory 섹션을 그대로 따름(중복 정의 안 함).
- **트리거**: `/memory-update`, "메모리 정리", "MEMORY 압축"
- **참고**: 평소 메모리 갱신은 매 대화마다 자동으로 일어나며, 이 스킬은 명시적 압축이 필요할 때만 호출.

### `/readme-project` — 프로젝트 README 생성
- **용도**: 프로젝트 폴더를 분석(패키지 매니페스트, lock 파일, CI 설정, 구조)해서 상위권 OSS 스타일 README.md 생성. 원칙은 "완전성보다 스캔 가능성" — hero 블록 + 가치 제안 + 핵심 기능 + 퀵스타트 순.
- **트리거**: `/readme-project <경로>`, "README 만들어", "README 다시 써줘"
- **참고**: 기본 출력은 영어, 한국어는 명시 요청. 내부 전용 레포나 단일 스크립트 프로젝트에는 과함.

### `/sync-claudebase` — 머신 간 동기화 점검
- **용도**: claudebase 레포 자체의 동기화 루틴. fetch → diff → 드리프트 체크 → install → 검증 → 버그 분류 순서를 **엄격히** 따르는 절차형 스킬. 오프라인이던 머신에 돌아왔을 때, 플러그인 드리프트가 의심될 때 사용.
- **트리거**: `/sync-claudebase`, "claudebase 동기화", "plugin drift", "install.sh 다시"
- **특징**: 비멱등 작업(커밋, 푸시, 새 파일 생성)은 반드시 물어보고 진행. 구 경로 `~/claude-settings` 레거시 클론을 감지해 처리.

## 3. 훅 (settings.json에 자동 배선됨)

설치하면 아래 훅들이 `~/claudebase/runtime/hooks/` 절대경로로 자동 등록된다. 별도 호출 불필요 — 언제 발동되는지만 알면 됨.

| 훅 | 이벤트 | 하는 일 |
|:---|:---|:---|
| `askuserquestion-guard.py` | PreToolUse (AskUserQuestion) | 빈/불완전한 AskUserQuestion 호출을 차단하고 수정 방법을 모델에게 안내 |
| `askuserquestion_retry.py` | Stop | `questions` 필드가 아예 빠진 호출(스키마 검증에서 먼저 거부되는 케이스)을 사후 감지해 즉시 재시도 유도 |
| `agent-routing-guard.py` | PreToolUse (Agent/Task) | 리서치성 작업을 리서치 전용이 아닌 서브에이전트로 보내는 오라우팅을 차단 |
| `detect_malformed_toolcall.py` | Stop | 툴콜 마크업이 텍스트로 새어나간 경우 감지, 네이티브 tool_use로 재발행 유도 |
| `fix_surrogate.py` | Stop + SessionStart | 트랜스크립트의 깨진 UTF-16 서로게이트를 U+FFFD로 자동 수리(API 400 방지). 수동 사용: `fix_surrogate.py --check/--fix FILE` |
| `hud-ensure.sh` | SessionStart | OMC HUD 래퍼가 플러그인 업데이트로 초기화됐을 때 로컬 커스터마이징(cyan dir/branch/model 표시) 재주입 |
| `askuserquestion_stats.py` | (수동 진단) | guard/retry 훅이 남긴 실패 로그(`.omc/logs/*.jsonl`) 집계: `python3 runtime/hooks/askuserquestion_stats.py` |
| `merge-project-hook.py` | (설치기 내부용) | 훅 조각을 프로젝트 `.claude/settings.json`에 멱등 병합하는 유틸 |
| `omc-reference-emit.py` | SessionStart (프로젝트 배포용) | 캐시된 OMC 레퍼런스 카탈로그를 세션 시작 시 주입 |

## 4. 설정 계층

- **`config/settings.json`** (git 추적, 전 머신 공통): 훅 배선, statusLine(OMC HUD), 플러그인 14개 + 마켓플레이스 4개, `model: sonnet`, `effortLevel: xhigh`, `language: 한국` 등.
- **`~/.claude/settings.local.json`** (gitignore, 머신 로컬): 머신별 오버라이드. Claude Code가 settings.json **위에** 병합하므로 개인 모델/플러그인/권한은 여기에 둔다. 예시는 `templates/settings.local.example.json`.
- **`config/CLAUDE.md`** (user-scope 행동 규칙): Think Before Coding, Simplicity First, Surgical Changes 등 6원칙 + OMC 오케스트레이션 공존 규칙.
- **`secrets/secrets.env`** (gitignore): MCP 서버 API 키. `cp secrets/secrets.example.env secrets/secrets.env` 후 편집하고 `install.sh` 재실행하면 `~/.claude/mcp.json`이 렌더링됨.

## 5. 템플릿 (`templates/`)

새 프로젝트의 `.claude/` 초기화용 보일러플레이트: `project-CLAUDE.md`, `project-settings.json`, `project-gitignore`, `project-refactor-workflow.md`, `settings.local.example.json`. 필요할 때 프로젝트로 복사해서 사용.

## 6. 플러그인 하네스 사용법

설치된 플러그인 14개 중 주력 하네스들:

### oh-my-claudecode (OMC) — 멀티에이전트 오케스트레이션
프롬프트에 키워드를 쓰거나 슬래시 명령으로 호출:

| 명령/키워드 | 용도 |
|:---|:---|
| `autopilot: <목표>` | 목표만 주면 계획→구현→검증까지 자율 실행 |
| `ralph <작업>` | 완료될 때까지 자가 반복 루프 (끈질긴 재시도) |
| `/oh-my-claudecode:plan`, `ralplan` | 계획 수립 (ralplan은 ralph용 계획) |
| `/oh-my-claudecode:deep-dive` | 코드베이스 심층 분석 |
| `/oh-my-claudecode:debug` | 체계적 디버깅 세션 |
| `/oh-my-claudecode:ask` | 코드 수정 없이 질문만 |
| `/oh-my-claudecode:deepinit` | 프로젝트 CLAUDE.md 심층 생성 |
| `/oh-my-claudecode:hud` | statusLine HUD 설정 (`hud setup`, preset 변경) |
| `/oh-my-claudecode:omc-doctor` | OMC 설치 상태 진단 |
| `/oh-my-claudecode:remember` | 세션 학습 내용을 메모리에 저장 |
| `cancel` | 실행 중인 autopilot/ralph 중단 |

### oh-my-docs (omd) — 문서 생성 파이프라인
PPT/DOCX/HWPX 문서를 단계별로 생성. `docs-intake`(요구 수집) → `docs-plan`(구성 설계) → `docs-build`(빌드) → `docs-verify`(검증) → `docs-revise`(수정) 순서로 진행되며, `docs-pilot`이 전체 파이프라인을 자동 조율. 변환은 `docs-convert`, 번역은 `docs-translate`.
**주의**: Python venv 필요 — `~/.claude/.venv`에 설치돼 있으며, `settings.local.json`에 PATH 추가 필요 (아래 §8).

### oh-my-project (omp) — 프로젝트 관리
`omp-init`(프로젝트 구조 초기화), `omp-audit`(상태 감사), `omp-organize`(파일 정리), `omp-doc`(문서화), `omp-dataset`(데이터셋 관리), `omp-env`(환경 설정), `omp-doctor`(진단), `omp-learn`/`omp-codify`(학습 내용 성문화).

### oh-my-experiments (omx) — 실험 관리
`exp-init`(실험 초기화) → `exp-design`(실험 설계) → `exp-loop`(실행 루프) → `exp-analyze`(결과 분석). ROV 튜닝 실험 같은 반복 실험에 적합.

### 기타
- **superpowers** — 검증된 작업 절차(스킬) 모음, 자동 발동
- **ponytail** — 과잉 설계 방지 (`/ponytail-review`, "lazy mode", "yagni")
- **context7** — 라이브러리 최신 문서 조회 MCP
- **pyright-lsp / clangd-lsp** — Python/C++ 코드 인텔리전스
- **axlabs-mckinsey-pptx** — 맥킨지 스타일 PPT 생성

## 7. tmux 사용법 (prefix: `Ctrl+a`)

`~/.tmux.conf` → `shell/tmux.conf` 심링크. 기본 prefix가 `Ctrl+b`가 아니라 **`Ctrl+a`**로 변경돼 있음.

### 세션 기본
```bash
tmux new -s work      # 새 세션
tmux attach -t work   # 재접속
# Ctrl+a d            # 세션에서 분리(detach) — 작업은 계속 돌아감
```

### 키 바인딩 (모두 Ctrl+a 누른 후)
| 키 | 동작 |
|:---|:---|
| `o` / `e` | 창 분할: 위아래 / 좌우 (현재 디렉토리 유지) |
| `t` | 새 윈도우 |
| `w` | 창(pane) 닫기 — 마지막 창이면 윈도우 닫기 |
| `[` / `]` | 이전 / 다음 윈도우 |
| `n` | 윈도우 이름 변경 |
| `W` | 세션/윈도우 트리 뷰 |
| `s` | 모든 창에 동시 입력 토글 (sync panes) |
| `m` | 마우스 모드 on/off 토글 |
| `r` | tmux.conf 리로드 |
| `P` | 붙여넣기 |

### 복사/붙여넣기 (마우스)
- **드래그** → 자동으로 클립보드 복사 (xclip 설치됨)
- **더블클릭** → 단어 복사, **트리플클릭** → 줄 복사
- **가운데/우클릭** → 붙여넣기
- SSH 접속 시에도 OSC52로 로컬 PC 클립보드까지 복사됨
- VS Code 터미널 등에서 드래그가 먹통이면 `Ctrl+a m`으로 마우스 끄고 네이티브 선택 사용

### 복사 모드 (vi 스타일)
스크롤 올리면 자동 진입. `v` 선택 시작 → `y` 또는 `Enter` 복사.

## 8. 설치 후 남은 설정 (`~/.claude/settings.local.json`)

```json
{
  "env": { "PATH": "/home/hero/.claude/.venv/bin:$PATH" },
  "model": "claude-fable-5[1m]",
  "enabledPlugins": { "ponytail@ponytail": true },
  "extraKnownMarketplaces": { "ponytail": { "source": { "source": "github", "repo": "DietrichGebert/ponytail" } } }
}
```
- `PATH` — 문서 스킬(omd)이 venv의 python3를 찾도록
- `model` — 레포 기본값(sonnet) 대신 기존에 쓰던 모델 유지 (선택)
- `ponytail` — drift-kept 상태라 등록해두면 `--prune` 시에도 유지

**미설치 항목**: `oh-my-scholar`(비공개 저장소 — `gh auth login` 후 설치), `gemini` CLI(`/gen-image`용 — `sudo npm install -g @google/gemini-cli`).

## 9. 문제가 생기면

- 설정이 안 먹음 → `ls -la ~/.claude/settings.json`으로 심링크가 살아있는지 확인, 끊겼으면 `install.sh` 재실행
- 플러그인 드리프트 → `/sync-claudebase` 실행
- statusLine이 템플릿 문자 그대로 나옴 → `jq` 미설치 (`sudo apt-get install -y jq`)
- 설치 전 설정 복원 → `installer/bin/restore-settings.sh` 또는 `~/.claude/backups/` 확인
- 상세 구조는 `~/claudebase/docs/ARCHITECTURE.md` 참고

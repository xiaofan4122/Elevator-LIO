#!/usr/bin/env bash
# Elevator-LIO 数据集下载脚本，只使用 huggingface_hub。

set -euo pipefail

HF_REPO="xiaofan0100/Elevator-LIO-Dataset"
HF_BASE_URL="https://huggingface.co/datasets/${HF_REPO}"
SJTU_URL="https://pan.sjtu.edu.cn/web/share/3792f963d0e41237c86303c1586a4ba1"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PROJECT_PARENT="$(cd -- "${PROJECT_ROOT}/.." && pwd)"
OUTPUT_DIR="${PROJECT_PARENT}/Elevator-LIO-Dataset"
SJTU_MODE=false
INCLUDE_COMMUNITY=false

# ---- 彩色输出 ----------------------------------------------------------------
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }
banner(){ echo -e "${CYAN}$*${NC}"; }

# ---- 帮助 --------------------------------------------------------------------
usage() {
    cat <<EOF
用法:
  $0 [选项]

默认行为:
  从 Hugging Face 下载 Elevator-LIO 主数据集到:
  ${OUTPUT_DIR}

选项:
  --output-dir DIR    指定数据集输出目录
  --include-community 同时下载 community_contributions/ 社区贡献数据
  --sjtu              显示上海交大云盘下载地址
  --help, -h          显示本帮助

示例:
  $0
  $0 --output-dir /data/Elevator-LIO-Dataset
  $0 --include-community
  $0 --sjtu

说明:
  默认只下载主数据集，不下载 community_contributions/。
  下载支持断点续传。中断后再次执行相同命令即可继续。
  脚本不会自动配置代理，会继承当前终端的代理环境变量。
  国内用户建议优先使用 --sjtu。
EOF
}

# ---- Hugging Face CLI ---------------------------------------------------------
find_hf_cli() {
    local command_name
    for command_name in hf huggingface-cli \
        "${HOME}/.local/bin/hf" "${HOME}/.local/bin/huggingface-cli"; do
        if command -v "${command_name}" &>/dev/null; then
            command -v "${command_name}"
            return
        fi
    done
    return 1
}

# ---- Hugging Face 下载 --------------------------------------------------------
hf_download() {
    local output_dir="$1"
    local hf_cli=""
    local download_args=()
    mkdir -p "${output_dir}"

    if ! hf_cli="$(find_hf_cli)"; then
        warn "未安装 huggingface_hub。"
        echo "将执行: python3 -m pip install --user --upgrade huggingface_hub"
        read -r -p "是否安装并继续？[Y/n] " answer
        if [[ "${answer:-Y}" =~ ^[Nn]$ ]]; then
            return 1
        fi
        python3 -m pip install --user --upgrade huggingface_hub
        hash -r
        hf_cli="$(find_hf_cli)" || return 1
    fi

    info "下载仓库: ${HF_REPO}"
    info "保存目录: ${output_dir}"
    if ${INCLUDE_COMMUNITY}; then
        info "包含社区贡献数据: community_contributions/"
    else
        info "默认跳过社区贡献数据；如需下载请添加 --include-community"
        download_args+=(--exclude "community_contributions/*")
    fi
    info "支持断点续传；如下载中断，重新执行本命令即可。"
    if [[ -n "${HTTPS_PROXY:-${https_proxy:-}}" || -n "${ALL_PROXY:-${all_proxy:-}}" ]]; then
        info "检测到代理环境变量，下载将继承当前代理设置。"
    else
        warn "未检测到代理环境变量；国内网络访问 Hugging Face 可能较慢或连接超时。"
        info "可先手动 export HTTPS_PROXY/HTTP_PROXY"
    fi
    echo ""
    info "正在启动 Hugging Face 下载..."

    HF_HUB_DISABLE_XET=1 HF_HUB_DISABLE_PROGRESS_BARS=0 \
    "${hf_cli}" download "${HF_REPO}" \
        --repo-type dataset \
        --local-dir "${output_dir}" \
        "${download_args[@]}" \
        --max-workers 1 &
    local download_pid=$!
    local last_progress_bytes
    local last_progress_time="${SECONDS}"
    local timeout_reminded=false
    last_progress_bytes="$(du -sb "${output_dir}" 2>/dev/null | awk '{print $1}')"

    trap 'kill "${download_pid}" 2>/dev/null || true' INT TERM
    while kill -0 "${download_pid}" 2>/dev/null; do
        local downloaded_bytes
        downloaded_bytes="$(du -sb "${output_dir}" 2>/dev/null | awk '{print $1}')"

        if [[ "${downloaded_bytes:-0}" -gt "${last_progress_bytes:-0}" ]]; then
            last_progress_bytes="${downloaded_bytes}"
            last_progress_time="${SECONDS}"
            timeout_reminded=false
        elif (( SECONDS - last_progress_time >= 10 )) && ! ${timeout_reminded}; then
            warn "已连续 10 秒没有下载进展"
            info "请检查代理设置；"
            timeout_reminded=true
        fi
        sleep 5
    done

    local download_status=0
    wait "${download_pid}" || download_status=$?
    trap - INT TERM
    if [[ "${download_status}" -ne 0 ]]; then
        error "下载失败或被中断，可重新运行脚本继续下载。"
        return "${download_status}"
    fi

    echo ""
    info "数据集下载完成: ${output_dir}"
}

# ---- 主入口 ------------------------------------------------------------------
main() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --output-dir)
                if [[ $# -lt 2 ]]; then
                    error "--output-dir 后需要提供目录"
                    echo ""
                    usage
                    exit 2
                fi
                if [[ -z "$2" ]]; then
                    error "--output-dir 后需要提供目录"
                    echo ""
                    usage
                    exit 2
                fi
                OUTPUT_DIR="$2"
                shift 2
                ;;
            --sjtu)        SJTU_MODE=true; shift ;;
            --include-community)
                INCLUDE_COMMUNITY=true
                shift
                ;;
            --help|-h)
                usage
                exit 0
                ;;
            *)
                error "未知参数: $1"
                echo ""
                usage
                exit 2
                ;;
        esac
    done

    info "输出目录: ${OUTPUT_DIR}"

    if ${SJTU_MODE}; then
        info "交大云盘地址:"
        info "  ${SJTU_URL}"
        echo ""
        info "请在浏览器中打开上述公开分享链接并下载，无需登录 SJTU 账号。"
        return
    fi

    banner "============================================"
    banner "  Elevator-LIO 数据集下载"
    banner "  ${HF_BASE_URL}"
    banner "============================================"
    echo ""
    info "国内用户如访问 Hugging Face 较慢，建议按 Ctrl+C 退出后执行:"
    info "  $0 --sjtu"
    echo ""
    hf_download "${OUTPUT_DIR}"
}

main "$@"

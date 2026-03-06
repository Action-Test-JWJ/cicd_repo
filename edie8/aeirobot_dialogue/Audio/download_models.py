import os
import logging
import shutil
import pathlib
import torch
from huggingface_hub import snapshot_download

# --- 다운로드 기본 경로 ---
# 사용자 홈 디렉토리 밑에 .aeirobot_models 폴더로 지정
MODELS_BASE_DIR = os.path.expanduser("~/.aeirobot_models")

# --- 모델 설정 ---
# 각 모델에 대한 설정을 딕셔너리 리스트로 관리
# download_type: 'huggingface', 'torch_hub', 'url' (추후 확장 가능)
# target_subdir: MODELS_BASE_DIR 내부에 생성될 하위 디렉토리명
MODELS_CONFIG = [
    {
        "name": "Language ID Model (VoxLingua107 ECAPA)",
        "repo_id": "speechbrain/lang-id-voxlingua107-ecapa",
        "download_type": "huggingface",
        "target_subdir": "lang-id-voxlingua107-ecapa",
    },
    {
        "name": "STT Model (Faster Whisper small ct2)",
        "repo_id": "Systran/faster-whisper-small",
        "download_type": "huggingface",
        "target_subdir": "faster-whisper-small-ct2",
    },
    {
        "name": "Silero VAD Model",
        "repo_id": "snakers4/silero-vad",  # torch.hub.load의 repo_or_dir
        "model_name_in_repo": "silero_vad",  # torch.hub.load의 model
        "download_type": "torch_hub",
        # target_subdir는 torch.hub가 repo_id 기반으로 자동 생성 (예: snakers4_silero-vad_master)
        # 이 스크립트에서는 MODELS_BASE_DIR를 torch.hub.set_dir()로 설정하여 관리
        "onnx": True, # Silero VAD의 경우 ONNX 사용 여부
    }
]

# 로깅 설정
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def resolve_symlinks_in_dir(directory_path):
    """
    Recursively finds all symlinks in a directory and replaces them with a copy of their target file.
    """
    logging.info(f"Checking for and resolving symlinks in {directory_path}...")
    resolved_count = 0
    # 디렉토리가 존재하지 않으면 경고 후 종료
    if not os.path.isdir(directory_path):
        logging.warning(f"Directory not found for symlink resolution: {directory_path}")
        return

    for root, _, files in os.walk(directory_path):
        for filename in files:
            filepath = pathlib.Path(root) / filename
            if filepath.is_symlink():
                try:
                    target_path = filepath.resolve(strict=True)
                    if target_path.is_file() and str(target_path) != str(filepath.absolute()):
                        logging.info(f"  Resolving symlink: {filepath} -> {target_path}")
                        temp_filepath = filepath.with_name(filepath.name + "_TEMP_COPY_RESOLVE")
                        shutil.copy2(target_path, temp_filepath)
                        filepath.unlink()
                        temp_filepath.rename(filepath)
                        resolved_count += 1
                    elif not target_path.exists():
                        logging.warning(f"  Symlink {filepath} points to a non-existent target: {os.readlink(str(filepath))}. Skipping.")
                except FileNotFoundError:
                    logging.warning(f"  Symlink {filepath} is broken (target not found: {os.readlink(str(filepath))}). Skipping.")
                except Exception as e:
                    logging.error(f"  Error resolving symlink {filepath}: {e}")
    if resolved_count > 0:
        logging.info(f"Resolved {resolved_count} symlink(s) in {directory_path}.")
    else:
        logging.info(f"No symlinks needed resolution or found in {directory_path}.")

def _download_huggingface_model(config, model_base_dir):
    """Hugging Face Hub에서 모델을 다운로드하고 심볼릭 링크를 처리합니다."""
    model_dir_name = config["target_subdir"]
    model_target_path = os.path.join(model_base_dir, model_dir_name)
    repo_id = config["repo_id"]

    logging.info(f"대상 경로: {model_target_path}")
    if not os.path.exists(model_target_path):
        logging.info(f"모델 디렉토리가 존재하지 않아 새로 다운로드합니다: {model_target_path}")
        os.makedirs(model_target_path, exist_ok=True)
        snapshot_download(
            repo_id=repo_id,
            local_dir=model_target_path,
            # force_download=True, # 필요시 활성화
        )
        logging.info(f"{config['name']} 다운로드 완료: {model_target_path}")
    else:
        logging.info(f"모델 디렉토리가 이미 존재합니다. 다운로드를 건너<0xEB><0><0x8F>니다: {model_target_path}")

    resolve_symlinks_in_dir(model_target_path)

def _ensure_torch_hub_model(config, model_base_dir):
    """torch.hub를 통해 모델을 다운로드하거나 로컬 캐시 사용을 확인합니다."""
    # torch.hub는 model_base_dir 내에 repo_id 기반으로 자체적인 디렉토리 구조를 생성
    # 예: model_base_dir/snakers4_silero-vad_master/
    # 이 디렉토리에 대해 심볼릭 링크 처리는 일반적으로 불필요.
    torch.hub.set_dir(model_base_dir)
    logging.info(f"torch.hub 저장 경로를 {model_base_dir}로 설정합니다.")
    try:
        torch.hub.load(
            repo_or_dir=config["repo_id"],
            model=config["model_name_in_repo"],
            force_reload=False, # 이미 있으면 사용
            onnx=config.get("onnx", False)
        )
        logging.info(f"{config['name']}이(가) {model_base_dir} 내에 준비되었거나 다운로드되었습니다.")
        # torch.hub가 생성하는 정확한 경로를 알기 어렵고, 심볼릭 링크를 사용하지 않을 가능성이 높으므로
        # resolve_symlinks_in_dir는 여기서는 호출하지 않음.
        # 필요하다면, torch.hub.get_dir()과 config['repo_id']를 조합하여 경로를 추정하고 호출할 수 있음.
    except Exception as e:
        logging.error(f"{config['name']} 처리 중 오류 발생: {str(e)}")
        raise # 오류 발생 시 상위로 전파하여 전체 스크립트가 실패하도록 할 수 있음

def download_models():
    """
    MODELS_CONFIG에 정의된 모든 모델을 다운로드하거나 로컬 버전을 확인합니다.
    """
    os.makedirs(MODELS_BASE_DIR, exist_ok=True)
    logging.info(f"모델 기본 저장 경로는 {MODELS_BASE_DIR} 입니다.")

    for config in MODELS_CONFIG:
        logging.info(f"--- {config['name']} 처리 시작 ---")
        try:
            if config["download_type"] == "huggingface":
                _download_huggingface_model(config, MODELS_BASE_DIR)
            elif config["download_type"] == "torch_hub":
                _ensure_torch_hub_model(config, MODELS_BASE_DIR)
            # elif config["download_type"] == "url": # 추후 직접 URL 다운로드 로직 추가
            #     _download_from_url(config, MODELS_BASE_DIR)
            else:
                logging.warning(f"알 수 없는 다운로드 타입입니다: {config['download_type']} ({config['name']}) ")
        except Exception as e:
            logging.error(f"{config['name']} 처리 중 최상위 오류 발생: {str(e)}")
        logging.info(f"--- {config['name']} 처리 완료 ---")

if __name__ == "__main__":
    download_models()

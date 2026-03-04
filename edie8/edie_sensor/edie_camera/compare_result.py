import cv2
import numpy as np
from skimage.metrics import structural_similarity as ssim

def compute_metrics(orig, undist, border=50):
    # 그레이스케일 변환
    g1 = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY).astype(np.float32)
    g2 = cv2.cvtColor(undist, cv2.COLOR_BGR2GRAY).astype(np.float32)

    # 전체 MSE, PSNR, SSIM
    mse_all = np.mean((g1 - g2)**2)
    h, w = g1.shape
    psnr_all = 10 * np.log10((255**2) / mse_all) if mse_all>0 else float('inf')
    ssim_all = ssim(g1, g2, data_range=255)

    # 가장자리(양 옆  border 픽셀) 부분만
    # top, bottom, left, right 영역을 모두 합칩니다.
    mask = np.zeros_like(g1, bool)
    mask[:border, :] = True
    mask[-border:, :] = True
    mask[:, :border] = True
    mask[:, -border:] = True

    diff = (g1 - g2)**2
    mse_edge = diff[mask].mean()
    psnr_edge = 10 * np.log10((255**2) / mse_edge) if mse_edge>0 else float('inf')
    # SSIM은 ROI만 계산할 수 없으니, 전체 SSIM과 edge MSE/PSNR만 비교해도 경향 파악에 유용합니다.

    return {
        'MSE_all': mse_all, 'PSNR_all': psnr_all, 'SSIM_all': ssim_all,
        'MSE_edge': mse_edge, 'PSNR_edge': psnr_edge
    }


def kannala_brandt_undistort(w, h):
    K = np.array([[1.9466066248715637e+02, 0.0,                 3.1416083999103677e+02],
                  [0.0,                 1.9494620197500788e+02, 2.2917570841700532e+02],
                  [0.0,                 0.0,                  1.0]])
    D = np.array([6.8256758074992219e-01,
                  -5.6806025148677955e-02,
                  -1.1138004488200955e-01,
                  3.2483633549490035e-02])
    R = np.eye(3, dtype=np.float64)

    # OpenCV fisheye 모듈의 Kannala–Brandt용 맵 생성
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K, D, (w, h), R,
        balance=0.0,         # balance=0.0 → 최대한 원본 시야 유지, 필요에 따라 조절
        new_size=(w, h),
        fov_scale=1.0
    )

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K, D, R, new_K,
        (w, h), cv2.CV_16SC2
    )
    return map1, map2

def pinhole_undistort(w, h):
    K = np.array([[1.9877153507177397e+02, 0.0,                 3.1223535436890785e+02],
                  [0.0,                 1.9951877002508613e+02, 2.2858368564103529e+02],
                  [0.0,                 0.0,                  1.0]])
    D = np.array([1.1673254606151540e-01,
                  -5.9875516968327193e-02,
                  -1.3793457190557877e-03,
                  -2.7583588802807630e-03])
    R = np.eye(3, dtype=np.float64)

    newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), alpha=0)
    map1, map2 = cv2.initUndistortRectifyMap(K, D, None, newK, (w, h), cv2.CV_16SC2)

    return map1, map2

def ov9281_pinhole_undistort(w, h):
    K = np.array([[501.10980011376125, 0.0,                  312.37587945691126],
                  [0.0,                501.76249532165775,   216.4203586169687],
                  [0.0,                0.0,                  1.0]])
    D = np.array([-0.357572775890401,
                  0.13705793098060928,
                  0.0017587111917320003,
                  0.00023384401569657455])

    R = np.eye(3, dtype=np.float64)

    newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), alpha=0)
    map1, map2 = cv2.initUndistortRectifyMap(
        K, D, R, newK, (w, h), cv2.CV_16SC2
    )

    return map1, map2

def ov9281_equidistant_undistort(w, h):
    # intrinsics (fx, fy, cx, cy)
    K = np.array([
        [501.6599892305741, 0.0,                   312.4851520286072],
        [0.0,                502.3462642638363,    220.03945658593466],
        [0.0,                0.0,                  1.0]
    ])

    # equidistant distortion coefficients (k1, k2, k3, k4)
    D = np.array([
        -0.04724961943636571,
        0.01377189343551785,
        -0.0730651490495297,
        0.08874459421272476
    ])

    R = np.eye(3, dtype=np.float64)

    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K, D, (w, h), R,
        balance=0.0,        # 0.0: 최대한 원본 시야 유지, 1.0: 왜곡 줄이기
        new_size=(w, h),
        fov_scale=1.0
    )

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K, D, R, new_K, (w, h), cv2.CV_16SC2
    )

    return map1, map2
def equidistant_undistort(w, h):
    K = np.array([
        [199.535, 0.0, 305.709],
        [0.0, 200.145, 236.443],
        [0.0, 0.0, 1.0]
    ])
    D = np.array([0.660675, 0.032225, -0.262943, 0.114609])
    R = np.eye(3, dtype=np.float64)

    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K, D, (w, h), R,
        balance=0.0,         # balance=0.0 → 최대한 원본 시야 유지, 필요에 따라 조절
        new_size=(w, h),
        fov_scale=1.0
    )

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K, D, R, new_K, (w, h), cv2.CV_16SC2
    )

    return map1, map2
    
def main():
    cap = cv2.VideoCapture('/dev/video2')
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    # 첫 프레임으로 크기 얻어서 맵 미리 계산
    ret, frame = cap.read()
    if not ret:
        print("첫 프레임을 읽을 수 없습니다.")
        return

    h, w = frame.shape[:2]
    # map1, map2 = ov9281_pinhole_undistort(w, h)
    map1, map2 = ov9281_equidistant_undistort(w, h)
    # map1, map2 = kannala_brandt_undistort(w, h)
    # map1, map2 = equidistant_undistort(w, h)
    # map1, map2 = pinhole_undistort(w, h)

    # 이미 한 번 읽었으니 루프 시작
    while True:
        undistorted = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)
        metrics = compute_metrics(frame, undistorted, border=50)
        # 콘솔에 출력
        print(f"All — MSE: {metrics['MSE_all']:.2f}, PSNR: {metrics['PSNR_all']:.2f} dB, SSIM: {metrics['SSIM_all']:.4f}")
        print(f"Edge — MSE: {metrics['MSE_edge']:.2f}, PSNR: {metrics['PSNR_edge']:.2f} dB\n")

        cv2.imshow('undistorted', undistorted)
        cv2.imshow('frame', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        # 다음 프레임
        ret, frame = cap.read()
        if not ret:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
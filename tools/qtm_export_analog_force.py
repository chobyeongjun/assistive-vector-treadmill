"""
QTM Export — Analog + Force Plate만 자동 export

작동:
  1) qtm.settings.export.tsv 설정: 3D/Skeleton 등 끄고 analog + force만 ON
  2) qtm.settings.processing.set_export_tsv: reprocess 시 자동 export
  3) 각 .qtm 파일 열기 → process → 자동 .tsv 생성 → 닫기

출력:
  - 각 .qtm 옆에 동일 이름의 .tsv 파일 생성 (analog + force만)
  - device 여러 개면 _a01, _a02, _f01 형태로 자동 분리
  - QTM에선 입력 폴더에 .tsv 생성 → 사용자가 후에 02_Organized로 이동 가능

사용법:
  QTM Python Console에서:
    exec(open(r"D:\\path\\to\\qtm_export_analog_force.py").read())
"""
import qtm
import os

# ============================================================
# 설정
# ============================================================
DATA_ROOT = r"D:\data\260507_CBJ\01_Raw\Motion"
SESSIONS  = ["S1", "S2", "S3"]
INCLUDE_REFERENCE = False  # static, MVC, normal walking 도 export 할지

# ============================================================
# Export 채널 설정 — analog + force ONLY
# ============================================================
def configure_export():
    print("="*60)
    print("Export 설정: Analog + Force만 (다른 채널 OFF)")
    print("="*60)
    e = qtm.settings.export.tsv

    # 끄기
    e.set_export_2d(False)
    e.set_export_3d(False)        # 3D 마커 X
    e.set_export_6d(False)        # 6DOF rigid body X
    e.set_export_skeleton(False)  # skeleton segment X
    e.set_export_eye(False)
    e.set_export_event(False)
    e.set_export_point_type(False)

    # 켜기
    e.set_export_analog(True)     # ★ analog (A7 sync, EMG 등)
    e.set_export_force(True)      # ★ force plate

    # 헤더
    e.set_export_time(True)       # timestamp 포함
    e.set_export_file_header(True)
    e.set_export_column_header(True)

    # Filtering
    e.set_exclude_unidentified(True)
    e.set_exclude_empty(True)

    print("  ✓ analog + force ONLY")
    print("  ✓ 3D, 6D, skeleton, eye, event 끔")
    print("  ✓ time + headers 포함")

    # Processing pipeline에 TSV export 포함
    qtm.settings.processing.set_export_tsv("reprocess", True)
    print("  ✓ reprocess 시 자동 TSV export 트리거")

# ============================================================
# 일괄 처리
# ============================================================
def process_all():
    qtm_files = []
    for sess in SESSIONS:
        sess_dir = os.path.join(DATA_ROOT, sess)
        if not os.path.isdir(sess_dir):
            print(f"  ⚠ {sess_dir} 없음")
            continue
        for f in sorted(os.listdir(sess_dir)):
            if not f.endswith('.qtm'):
                continue
            # Reference 제외 (옵션)
            if not INCLUDE_REFERENCE:
                low = f.lower()
                if any(p in low for p in ('mvc', 'static', 'normal')):
                    continue
            qtm_files.append((sess, os.path.join(sess_dir, f), f))

    print(f"\n총 {len(qtm_files)}개 파일 export")
    print("-"*60)

    success, failed = 0, []
    for i, (sess, path, fname) in enumerate(qtm_files, 1):
        try:
            qtm.gui.open_file(path)
            qtm.gui.process_measurement()  # → 자동 TSV export 발생
            qtm.gui.save_file()             # .qtm도 다시 저장 (선택)
            qtm.gui.close_file()
            print(f"[{i:2}/{len(qtm_files)}] ✓ {sess}/{fname}")
            success += 1
        except Exception as ex:
            failed.append((fname, str(ex)))
            print(f"[{i:2}/{len(qtm_files)}] ✗ {fname}: {ex}")

    print("-"*60)
    print(f"성공: {success}/{len(qtm_files)}")
    if failed:
        print(f"\n실패 {len(failed)}개:")
        for fname, err in failed:
            print(f"  ✗ {fname}: {err}")

# ============================================================
# 실행
# ============================================================
if __name__ == "__main__":
    configure_export()
    process_all()

    print("\n" + "="*60)
    print("완료. 각 세션 폴더에 .tsv 파일 생성됨")
    print("="*60)
    print(f"""
출력 파일 형식 (device 1개 가정):
  {DATA_ROOT}/S1/...high_0_01.qtm
        ↓
  {DATA_ROOT}/S1/...high_0_01_a.tsv   ← analog (A7 sync 등)
  {DATA_ROOT}/S1/...high_0_01_f.tsv   ← force plate (있을 시)

device 여러 개면:
  ...high_0_01_a01.tsv, _a02.tsv
  ...high_0_01_f01.tsv, _f02.tsv
""")

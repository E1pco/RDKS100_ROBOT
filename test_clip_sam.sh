#!/bin/bash
# CLIP检索 + DOSOD检测 + EdgeSAM分割 测试脚本
#
# 使用方法:
#   Step 1: 索引图片到CLIP数据库
#     bash test_clip_sam.sh store
#
#   Step 2: 启动完整管线 (检索+检测+分割)
#     bash test_clip_sam.sh query "a cat"
#
#   或者跳过CLIP，直接检测+分割:
#     bash test_clip_sam.sh detect

set -e
cd /home/sunrise/fast_ws

case "${1:-help}" in
  store)
    echo "[Step 1] CLIP存储模式 - 索引测试图片..."
    echo "  图片目录: test_images/"
    echo "  数据库: /tmp/clip_sam_test.db"
    ros2 launch mono_edgesam sam_with_clip.launch.py \
      clip_mode:=0 \
      img_folder:=/home/sunrise/fast_ws/test_images \
      clip_storage_folder:=/home/sunrise/fast_ws/test_images \
      clip_db_file:=/tmp/clip_sam_test.db
    ;;
  query)
    TEXT="${2:-a cat}"
    echo "[Step 2] CLIP查询 + DOSOD检测 + EdgeSAM分割"
    echo "  检索文本: ${TEXT}"
    ros2 launch mono_edgesam sam_with_clip.launch.py \
      clip_mode:=1 \
      clip_text:="${TEXT}" \
      img_folder:=/home/sunrise/fast_ws/test_images \
      clip_storage_folder:=/home/sunrise/fast_ws/test_images \
      clip_result_folder:=/tmp/clip_sam_result \
      clip_db_file:=/tmp/clip_sam_test.db \
      clip_topk:=3
    ;;
  detect)
    echo "[Direct] DOSOD检测 + EdgeSAM分割 (跳过CLIP)"
    ros2 launch mono_edgesam sam_with_clip.launch.py \
      clip_mode:=0 \
      img_folder:=/home/sunrise/fast_ws/test_images
    ;;
  *)
    echo "用法: bash test_clip_sam.sh [store|query|detect] [检索文本]"
    echo ""
    echo "  store             索引图片到CLIP数据库"
    echo "  query [text]      CLIP检索 + 检测 + 分割"
    echo "  detect            直接检测 + 分割 (跳过CLIP)"
    echo ""
    echo "测试图片目录: /home/sunrise/fast_ws/test_images/"
    ls /home/sunrise/fast_ws/test_images/*.jpg 2>/dev/null || echo "  (无图片)"
    ;;
esac

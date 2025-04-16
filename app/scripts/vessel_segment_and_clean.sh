#!/bin/bash
cpp_program="../../build/app/VesselSegment"
inputfolder="/media/wlsdzyzl/DATA/datasets/pcd/imageCAS/output/"

xyzr_files=$(ls "$inputfolder/xyzr")
echo ls "$inputfolder/xyzr"
total_time=0
file_count=0
mkdir "$inputfolder/clean_segment"
for file in $xyzr_files; do
  filename=$(basename "$file" ".xyzr")
  # 指定输出文件路径
  inputxyzr="$inputfolder/xyzr/${file}"
  inputpcd="$inputfolder/surface/${filename}.ply"
  output_file="$inputfolder/clean_segment/${filename}.ply"
  start_time=$(date +%s.%N)
  echo $cpp_program "$inputpcd" "$inputxyzr" "$output_file" "2" "0.15" "10" "0" "1"
  # 使用编译好的C++程序处理文件，并指定输出文件路径
  $cpp_program "$inputpcd" "$inputxyzr" "$output_file" "2" "0.15" "10" "0" "1"
  end_time=$(date +%s.%N)
  duration=$(echo "$end_time - $start_time" | bc)
  total_time=$(echo "$total_time + $duration" | bc)
  file_count=$((file_count + 1))
done
average_time=$(echo "scale=2; $total_time / $file_count" | bc)
echo "总共处理了 $file_count 个文件"
echo "平均处理时间为 $average_time 秒"
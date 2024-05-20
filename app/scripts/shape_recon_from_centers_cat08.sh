#!/bin/bash

cpp_program="../../build/app/CenterLine2SDF"
inputfolder="/media/wlsdzyzl/DATA/datasets/CAT08/xyzr"
outputfolder="/media/wlsdzyzl/DATA/datasets/CAT08/mesh_from_xyzr_ours_fast"
files=$(find "$inputfolder" -type f)
total_time=0
file_count=0
for file in $files; do
  # 获取文件名和后缀
  filename=$(basename "$file")
  extension="${filename##*.}"

  # 去除后缀的文件名
  filename_without_extension="${filename%.*}"

  # 指定输出文件路径
  output_file="${outputfolder}/${filename_without_extension}.ply"
  start_time=$(date +%s.%N)
  # 使用编译好的C++程序处理文件，并指定输出文件路径
  $cpp_program "$file" "$output_file" "0.005" "0" "0.75" "-1" "1"
  end_time=$(date +%s.%N)
  duration=$(echo "$end_time - $start_time" | bc)
  total_time=$(echo "$total_time + $duration" | bc)
  file_count=$((file_count + 1))
done
average_time=$(echo "scale=2; $total_time / $file_count" | bc)
echo "总共处理了 $file_count 个文件"
echo "平均处理时间为 $average_time 秒"
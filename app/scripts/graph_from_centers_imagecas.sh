#!/bin/bash

cpp_program="../../build/app/CenterLine2Graph"
inputfolder="/media/wlsdzyzl/DATA/datasets/imageCAS/dataset_nii_1000/skeleton_right_xyzr"
# inputfolder="/media/wlsdzyzl/DATA/datasets/imageCAS/dataset_nii_1000/ot_skeleton/"
outputfolder="/media/wlsdzyzl/DATA/datasets/imageCAS/dataset_nii_1000/skeleton_right_graph"

files=$(find "$inputfolder" -type f)
total_time=0
file_count=0
for file in $files; do
  filename=$(basename "$file" ".xyzr")

  # 指定输出文件路径
  output_file="${outputfolder}/${filename}.ply"
  start_time=$(date +%s.%N)
  echo $cpp_program "$file" "$output_file" "1"
  # 使用编译好的C++程序处理文件，并指定输出文件路径
  $cpp_program "$file" "$output_file" "1"
  end_time=$(date +%s.%N)
  duration=$(echo "$end_time - $start_time" | bc)
  total_time=$(echo "$total_time + $duration" | bc)
  file_count=$((file_count + 1))
done
average_time=$(echo "scale=2; $total_time / $file_count" | bc)
echo "总共处理了 $file_count 个文件"
echo "平均处理时间为 $average_time 秒"
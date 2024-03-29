#!/bin/bash

cpp_program="../../build/app/CenterLine2Graph"
# inputfolder="/media/wlsdzyzl/DATA1/datasets/imageCAS/dataset_nii_1000/output/xyzr/"
inputfolder="/media/wlsdzyzl/DATA1/datasets/imageCAS/dataset_nii_1000/ot_skeleton_unfixed_n/"
outputfolder="/media/wlsdzyzl/DATA1/datasets/imageCAS/dataset_nii_1000/output/graph_ot/"

files=$(find "$inputfolder" -type d)
total_time=0
file_count=0
for file in $files; do
  # 获取文件名和后缀
  filename=$(basename "$file" "_skeletonization")

  # 去除后缀的文件名
  # filename_without_extension="${filename%\_*}"

  # 指定输出文件路径
  output_file="${outputfolder}/${filename}.ply"
  start_time=$(date +%s.%N)
  # 使用编译好的C++程序处理文件，并指定输出文件路径
  echo $cpp_program "$file/skeleton.xyzr" "$output_file" "0.0"
  $cpp_program "$file/skeleton.xyzr" "$output_file" "0.0"
  end_time=$(date +%s.%N)
  duration=$(echo "$end_time - $start_time" | bc)
  total_time=$(echo "$total_time + $duration" | bc)
  file_count=$((file_count + 1))
done
average_time=$(echo "scale=2; $total_time / $file_count" | bc)
echo "总共处理了 $file_count 个文件"
echo "平均处理时间为 $average_time 秒"
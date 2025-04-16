#!/bin/bash

cpp_program="../../build/app/Mesh2Indicator"
inputfolder="/media/wlsdzyzl/DATA/datasets/siqi/mesh_to_skeleton/Diffusion/mesh"
outputfolder="/media/wlsdzyzl/DATA/datasets/siqi/mesh_to_skeleton/Diffusion/npy"
subfolders=$(ls "$inputfolder")

for sfolder in ${subfolders}; do
  files=$(ls "$inputfolder/$sfolder")
  total_time=0
  file_count=0
  for file in $files; do
    extension="${file##*.}"

    # 去除后缀的文件名
    filename_without_extension="${file%.*}"

    # 指定输出文件路径
    input_file="${inputfolder}/${sfolder}/${file}"
    output_file="${outputfolder}/${sfolder}/${filename_without_extension}.npy"
    start_time=$(date +%s.%N)
    # 使用编译好的C++程序处理文件，并指定输出文件路径
    $cpp_program "$input_file" "$output_file" "1" "0.005" "0.05" "0"
    end_time=$(date +%s.%N)
    duration=$(echo "$end_time - $start_time" | bc)
    total_time=$(echo "$total_time + $duration" | bc)
    file_count=$((file_count + 1))
  done
  average_time=$(echo "scale=2; $total_time / $file_count" | bc)
  echo "总共处理了 $file_count 个文件"
  echo "平均处理时间为 $average_time 秒"
done
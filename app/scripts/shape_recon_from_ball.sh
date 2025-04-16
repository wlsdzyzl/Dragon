#!/bin/bash


python_program_ballpivoting="./ballpivoting_recon.py"
inputfolder="/media/wlsdzyzl/DATA/datasets/siqi/mesh_to_skeleton/Diffusion/pcd"
outputfolder="/media/wlsdzyzl/DATA/datasets/siqi/mesh_to_skeleton/Diffusion/mesh"
mkdir "${outputfolder}"
subfolders=$(ls "$inputfolder")
for sfolder in ${subfolders}; do
  files=$(ls "$inputfolder/$sfolder")
  total_time=0
  file_count=0
  mkdir "${outputfolder}/${sfolder}"
  for file in $files; do
    # 获取文件名和后缀
    extension="${file##*.}"

    # 去除后缀的文件名
    filename_without_extension="${file%.*}"
    input_file="${inputfolder}/${sfolder}/${file}"
    output_file="${outputfolder}/${sfolder}/${filename_without_extension}.ply"
    
    start_time=$(date +%s.%N)
    echo python $python_program_ballpivoting "$input_file" "$output_file"
    python $python_program_ballpivoting "$input_file" "$output_file"
    end_time=$(date +%s.%N)
    duration=$(echo "$end_time - $start_time" | bc)
    total_time=$(echo "$total_time + $duration" | bc)
    file_count=$((file_count + 1))
  done
  average_time=$(echo "scale=2; $total_time / $file_count" | bc)
  echo "总共处理了 $file_count 个文件"
  echo "平均处理时间为 $average_time 秒"
done
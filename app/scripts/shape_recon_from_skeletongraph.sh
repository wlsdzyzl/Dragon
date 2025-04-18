#!/bin/bash

cpp_program="../../build/app/SkeletonGraph2SDF"
inputfolder="//media/wlsdzyzl/DATA/datasets/siqi/imagecas/"
outputfolder="/media/wlsdzyzl/DATA/datasets/siqi/imagecas_final"

# subfolders=("cow_gen" "cow_recon_test" "intra_gen" "intra_recon_test" "intra_vae_gen" "intra_vae_recon_test" "march_gen" "march_recon_test")
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

    # 指定输出文件路径
    input_file="${inputfolder}/${sfolder}/${file}"
    output_file="${outputfolder}/${sfolder}/${filename_without_extension}.ply"
    
    start_time=$(date +%s.%N)
    # 使用编译好的C++程序处理文件，并指定输出文件路径
    echo $cpp_program "$input_file" "$output_file" "0.005" "-1" "1"
    $cpp_program "$input_file" "$output_file" "0.005" "-1" "1"
    end_time=$(date +%s.%N)
    duration=$(echo "$end_time - $start_time" | bc)
    total_time=$(echo "$total_time + $duration" | bc)
    file_count=$((file_count + 1))
  done
  average_time=$(echo "scale=2; $total_time / $file_count" | bc)
  echo "总共处理了 $file_count 个文件"
  echo "平均处理时间为 $average_time 秒"
done
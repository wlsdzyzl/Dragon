#!/bin/bash
cpp_program="../../build/app/VesselSegment"
inputfolder="/media/wlsdzyzl/DATA/datasets/siqi/mesh_to_skeleton/TreeDiffusion/output"

subfolders=$(ls "$inputfolder")
mkdir "${inputfolder}/../to_siqi"
for sfolder in ${subfolders}; do
  xyzr_files=$(ls "$inputfolder/$sfolder/xyzr")
  echo ls "$inputfolder/$sfolder/xyzr"
  total_time=0
  file_count=0
  mkdir "$inputfolder/$sfolder/segment"
  for file in $xyzr_files; do
    filename=$(basename "$file" ".xyzr")
    # 指定输出文件路径
    inputxyzr="$inputfolder/$sfolder/xyzr/${file}"
    inputpcd="$inputfolder/$sfolder/surface/${filename}.ply"
    output_file="$inputfolder/$sfolder/segment/${filename}.ply"
    start_time=$(date +%s.%N)
    echo $cpp_program "$inputpcd" "$inputxyzr" "$output_file" "2" "0.001" "2" "0"
    # 使用编译好的C++程序处理文件，并指定输出文件路径
    $cpp_program "$inputpcd" "$inputxyzr" "$output_file" "2" "0.001" "2" "0"
    end_time=$(date +%s.%N)
    duration=$(echo "$end_time - $start_time" | bc)
    total_time=$(echo "$total_time + $duration" | bc)
    file_count=$((file_count + 1))
  done
  
  mkdir "${inputfolder}/../to_siqi/${sfolder}"
  mkdir "${inputfolder}/../to_siqi/${sfolder}/skeleton"
  mkdir "${inputfolder}/../to_siqi/${sfolder}/radius"
  echo mv $inputfolder/$sfolder/segment/*.skeleton.ply ${inputfolder}/../to_siqi/${sfolder}/skeleton --verbose
  mv $inputfolder/$sfolder/segment/*.skeleton.ply ${inputfolder}/../to_siqi/${sfolder}/skeleton --verbose
  echo mv $inputfolder/$sfolder/segment/*.radius ${inputfolder}/../to_siqi/${sfolder}/radius --verbose
  mv $inputfolder/$sfolder/segment/*.radius ${inputfolder}/../to_siqi/${sfolder}/radius --verbose
  average_time=$(echo "scale=2; $total_time / $file_count" | bc)
  echo "总共处理了 $file_count 个文件"
  echo "平均处理时间为 $average_time 秒"
done
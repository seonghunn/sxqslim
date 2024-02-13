#!/bin/bash

# 입력 및 출력 디렉터리 지정
input_dir="/Users/seonghun/Documents/study/research/sulab/code/MyCode/QEM-codebase/model/input/thingi10k_98_mixed"
output_dir="/Users/seonghun/Documents/study/research/sulab/code/MyCode/QEM-codebase/model/output/thingi10k_result"

# 출력 디렉터리가 없으면 생성
if [ ! -d "$output_dir" ]; then
    mkdir -p "$output_dir"
fi

# 입력 디렉터리 내의 모든 .obj 파일에 대해 반복
for input_file in "$input_dir"/1088281.obj; do
    # 입력 파일 이름 추출 (확장자 제외)
    filename=$(basename -- "$input_file")
    filename="${filename%.*}"

    # 출력 파일 이름과 경로 설정
    output_file="$output_dir/${filename}" # 확장자는 필요에 따라 조정

    # QSlim 실행
    ./build/QSlim "$input_file" "$output_file" 0.5
done

echo "모든 파일이 처리되었습니다."

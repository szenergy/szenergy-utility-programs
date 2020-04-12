
for file in "$(pwd)/"*.bag
do
  #[[ -f "$file" ]] && touch "${file%.*}".txt
  [[ -f "$file" ]] && rosbag info "${file%.*}.bag" >> ${file%.*}.txt
  [[ -f "$file" ]] && echo "${file%.*} >> done"
done


# "$file" >> fajlnev.kiterj
# "${file%.*}" >> fajlnev
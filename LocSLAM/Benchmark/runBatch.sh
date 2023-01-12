#!/bin/bash

set -eo pipefail

cfg_ext="yaml"
if [ "$#" -lt 1 ]; then
  echo "Usage: ${0} pathToConfigFile.${cfg_ext}"
  exit 1
fi

if [ "$#" -ge 2 ]; then
  exp_name="${2}"
else
  exp_name="experiments_locslam"
fi

cur_dir="$(pwd)"
src_dir="$( cd "$( dirname "${0}" )" >/dev/null 2>&1 && pwd )"
cmd="${src_dir}/Examples/Generalized/generalized_CMU"

cfg_src_dir="$src_dir/$(dirname $1)"
cfg_src_fn="$(basename ${1%.${cfg_ext}})"


if [ "$cfg_src_fn" == "CMU" ]; then
  runs=(1 2 3 4 5 6 7 8 9 10 12)
  # minIntervals=(0 0.5 2 5 10)
  # gyroBias=(1 2 5)
  minIntervals=(0)
  gyroBias=(1 2 5)
  filter_cmd="${src_dir}/Tools/select_test_images.py ${src_dir}/Tools/testimages_CMU.txt %s %s"
elif [ "$cfg_src_fn" == "RC" ]; then
  runs=$( seq 1 408 )
#  runs=$( seq 1 414 )
  minIntervals=(0)
  gyroBias=(0)
  filter_cmd="${src_dir}/Tools/select_test_images2.py ${src_dir}/Tools/testimages_RC2.txt %s %s"
elif [ "$cfg_src_fn" == "RC_Route" ]; then
  runs=(1 2 3 4 5 6)
#  runs=$( seq 1 414 )
  minIntervals=(0)
  gyroBias=(1)
  filter_cmd=""
else
  echo "First parameter should be either a CMU.${cfg_ext} or RC.${cfg_ext} file."
  exit 1
fi

LC_ALL=en_US.UTF-8
max_parallel_ps=12
target_dir="${exp_name}_$(date +%F_%H-%M-%S)"
cfg_dst_dir="${cfg_src_dir}/${target_dir}"

fixed_map_orig_path=$(grep -E "^FixedMap\.BasePath" "${cfg_src_dir}/${cfg_src_fn}.${cfg_ext}" | sed 's/^.*:\w*//')
fixed_map_base_path=$(dirname ${fixed_map_orig_path})

mkdir -p "${cfg_dst_dir}"
printf "all : " > "${cfg_dst_dir}/Makefile_head"
cat /dev/null > "${cfg_dst_dir}/Makefile_body"

count=0
for fixed_map_path in ${fixed_map_orig_path}; do #If just using matches given in the yaml file
# for fixed_map_path in ${fixed_map_base_path}/*; do #If iterating over all types of matches.
  fixed_map_dir=$(basename $fixed_map_path)
  for bias in ${gyroBias[@]}; do
    for minInterval in ${minIntervals[@]}; do
      echo "Generating Makefile for ${fixed_map_dir}, bias=${bias}, min_interval=${minInterval}"
      grp_name1=$(printf "result_smoothed_%s_%s_bias%02d_int%f" "$cfg_src_fn" "${fixed_map_dir}" "${bias}" "${minInterval}")
      grp_name2=$(printf "result_tracked_%s_%s_bias%02d_int%f" "$cfg_src_fn" "${fixed_map_dir}" "${bias}" "${minInterval}")
      grp_cmd1="cat"
      grp_cmd2="cat"
      #Test_img filter for smoothed result
      printf "${grp_name1}_testimgs.txt : ${grp_name1}.txt\n" > "${cfg_dst_dir}/Makefile_${grp_name1}"
      printf "\t${filter_cmd}\n" "${grp_name1}.txt" "${grp_name1}_testimgs.txt" >> "${cfg_dst_dir}/Makefile_${grp_name1}"
      #Test_img filter for tracked result
      printf "${grp_name2}_testimgs.txt : ${grp_name2}.txt\n" > "${cfg_dst_dir}/Makefile_${grp_name2}"
      printf "\t${filter_cmd}\n" "${grp_name2}.txt" "${grp_name2}_testimgs.txt" >> "${cfg_dst_dir}/Makefile_${grp_name2}"
      #Targets for this configuration group
      printf "${grp_name1}.txt : " >> "${cfg_dst_dir}/Makefile_${grp_name1}"
      printf "${grp_name2}.txt : " >> "${cfg_dst_dir}/Makefile_${grp_name2}"
      for run in ${runs[@]}; do
        cfg_dst_fn=$(printf "%s_%s_run%03d_int%f_bias%02d.${cfg_ext}" $cfg_src_fn $fixed_map_dir $run $minInterval $bias)
        echo "Writing config file: ${cfg_dst_dir}/${cfg_dst_fn}"

        #OdoPredictAngleUncertainty should vary with bias
        old_prediction_angle_uncertainty=$( grep -E "^\s*Tracking\.OdoPredictAngleUncertainty" "${cfg_src_dir}/${cfg_src_fn}.${cfg_ext}" | sed -e 's/.*:\s*\(\S*\)$/\1/' )
        new_prediction_angle_uncertainty=$(awk "BEGIN {print $old_prediction_angle_uncertainty * sqrt($bias)}" )

        #Remove previous keys
        grep -v -E "Tracking\.OdoPredictAngleUncertainty|Data\.SequenceId|FixedMap\.BasePath|FixedMap\.MinInterval|Data\.OdometryFile|Load\.Offset|Load\.nFrames|Viewer\.UseViewer" "${cfg_src_dir}/${cfg_src_fn}.${cfg_ext}" > "${cfg_dst_dir}/${cfg_dst_fn}"

        #Write to cfg file
        echo "#--------------------------------" >> "${cfg_dst_dir}/${cfg_dst_fn}"
        echo "# Parameters set by runBatch.sh"   >> "${cfg_dst_dir}/${cfg_dst_fn}"
        echo "#--------------------------------" >> "${cfg_dst_dir}/${cfg_dst_fn}"
        printf "Load.Offset: 0\n" >> "${cfg_dst_dir}/${cfg_dst_fn}"
        printf "Load.nFrames: -1\n" >> "${cfg_dst_dir}/${cfg_dst_fn}"
        printf "Viewer.UseViewer: 0\n" >> "${cfg_dst_dir}/${cfg_dst_fn}"
        printf "Data.SequenceId: %d\n" $run >> "${cfg_dst_dir}/${cfg_dst_fn}"
        printf "FixedMap.MinInterval: %f\n" $minInterval >> "${cfg_dst_dir}/${cfg_dst_fn}"
        printf "Data.OdometryFile: odoMeas_bias%02d.txt\n" $bias >> "${cfg_dst_dir}/${cfg_dst_fn}"
        printf "FixedMap.BasePath: ${fixed_map_path}\n" >> "${cfg_dst_dir}/${cfg_dst_fn}"
        printf "Tracking.OdoPredictAngleUncertainty: ${new_prediction_angle_uncertainty}\n" >> "${cfg_dst_dir}/${cfg_dst_fn}"

        #Add command to Makefile
        slamOutput=$(printf "%s-debug-output.txt" "${cfg_dst_dir}/$cfg_dst_fn")
        lbOutput=$(printf "%s-cerr-output.txt" "${cfg_dst_dir}/$cfg_dst_fn")

        target1="${cfg_dst_fn}.bench-result.txt"
        target2="${cfg_dst_fn}.tracking.bench-result.txt"
        printf "${target1} " >> "${cfg_dst_dir}/Makefile_${grp_name1}"
        printf "${target1} : ${cfg_dst_fn} \n"  >> "${cfg_dst_dir}/Makefile_body"
        printf "\t${cmd} ${src_dir}/Vocabulary/ORBvoc.txt ${cfg_dst_dir} $cfg_dst_fn > $slamOutput 2> $lbOutput \n" >> "${cfg_dst_dir}/Makefile_body"
        # printf "\ttouch ${target1} ${target2}\n" >> "${cfg_dst_dir}/Makefile_body" #Dummy task, to test make file.
        grp_cmd1="${grp_cmd1} ${target1}"

        printf "${target2} " >> "${cfg_dst_dir}/Makefile_${grp_name2}"
        printf "${target2} : ${target1} \n"  >> "${cfg_dst_dir}/Makefile_body"

        grp_cmd2="${grp_cmd2} ${target2}"

        count=$(($count+1))
      done
      echo "" >> "${cfg_dst_dir}/Makefile_${grp_name1}"
      echo "" >> "${cfg_dst_dir}/Makefile_${grp_name2}"
      printf "\t${grp_cmd1} > ${grp_name1}.txt\n\n" >> "${cfg_dst_dir}/Makefile_${grp_name1}"
      printf "\t${grp_cmd2} > ${grp_name2}.txt\n\n" >> "${cfg_dst_dir}/Makefile_${grp_name2}"
      printf "${grp_name1}_testimgs.txt ${grp_name2}_testimgs.txt " >> "${cfg_dst_dir}/Makefile_head"
    done
  done
done

echo ""  >> "${cfg_dst_dir}/Makefile_head"
echo ".PHONY : all"  >> "${cfg_dst_dir}/Makefile_head"
echo ""  >> "${cfg_dst_dir}/Makefile_head"
cat "${cfg_dst_dir}/Makefile_head" "${cfg_dst_dir}/Makefile_result_"* "${cfg_dst_dir}/Makefile_body" > "${cfg_dst_dir}/Makefile"
rm "${cfg_dst_dir}/Makefile_head" "${cfg_dst_dir}/Makefile_result_"* "${cfg_dst_dir}/Makefile_body"

cd "${cfg_dst_dir}"
echo "Starting ${count} runs in ${max_parallel_ps} parallel processes."
make -j${max_parallel_ps} -k all
echo "Done."
cd "${cur_dir}"

#!/bin/bash
{
work_dir=`pwd`
type_name="DelayRange"
tunnel_name="flow_info"
data_dir=${work_dir}/data/${type_name}
# case_id Srlg: 1-pulse+, 2-delayksp, 3-costksp, 4-costksppulse+,
#      5-lagrangianksp, 6-CosePulse+
# case_id DRCR: 1-pulse+, 2-delayksp, 3-costksp, 4-costksppulse+,
#      5-lagrangianksp, 7-JPPulse+
case_id=1
# cd $work_dir
if [ ! -d $data_dir  ]
then
  echo "Wrong Dir, Please Check!"
  exit 1
fi
if [ ! -d ${work_dir}"/tunnels"  ]
then
  rm -rf tunnels
  mkdir ${work_dir}"/tunnels"
fi
output_folder=""
if [ $type_name == "SrlgDisjoint" ]
then
    echo "1"
    echo "type_name"
    if [ $case_id -eq 1 ]
    then
        output_folder=${type_name}"Pulse"
        echo $output_folder
    fi
    if [ $case_id -eq 2 ]
    then
        output_folder=${type_name}"DelayKsp"
        echo $output_folder
    fi
    if [ $case_id -eq 3 ]
    then
        output_folder=${type_name}"CostKsp"
        echo $output_folder
    fi
    if [ $case_id -eq 4 ]
    then
        output_folder=${type_name}"CostKspPulse"
        echo $output_folder
    fi
    if [ $case_id -eq 5 ]
    then
        output_folder=${type_name}"LagrangianKsp"
        echo $output_folder
    fi
    if [ $case_id -eq 6 ]
    then
        output_folder=${type_name}"CosePulse"
        echo $output_folder
    fi
fi

if [ $type_name == "DelayRange" ]
then
    echo "2"
    # echo "type_name"
    if [ $case_id -eq 1 ]
    then
        output_folder=${type_name}"Pulse"
        echo $output_folder
    fi
    if [ $case_id -eq 2 ]
    then
        output_folder=${type_name}"DelayKsp"
        echo $output_folder
    fi
    if [ $case_id -eq 3 ]
    then
        output_folder=${type_name}"CostKsp"
        echo $output_folder
    fi
    if [ $case_id -eq 4 ]
    then
        output_folder=${type_name}"CostKspPulse"
        echo $output_folder
    fi
    if [ $case_id -eq 5 ]
    then
        output_folder=${type_name}"LagrangianKsp"
        echo $output_folder
    fi
    if [ $case_id -eq 6 ]
    then
        output_folder=${type_name}"JPPulse"
        echo $output_folder
    fi
fi

make main
make split
num=0
mkdir ${output_folder}
# for Type in $(ls $data_dir)
# do
    for Case in $(ls $data_dir/)
    do
        for file in $(ls ${work_dir}"/tunnels/")
        do
            rm ${work_dir}"/tunnels/"${file}
        done

        newtoponame=${type_name}"_"${Case}"_topo.csv"
        cp -f $data_dir/${Case}/topo.csv $work_dir/$newtoponame
        newtunnelname=${type_name}"_"${Case}${tunnel_name}
        # echo $data_dir/${Case}/${tunnel_name}
        cp -f $data_dir/${Case}/${tunnel_name} $work_dir/$newtunnelname
        ./split $newtunnelname

        echo $work_dir/$newtunnelname
        echo "-----------------------------------------------------------"
        for file in $(ls ${work_dir}"/tunnels/")
        do
            name=${file%%.*}
            type=${name##*_}
            case=${name%_*}
            echo $file
            timeout 10s ./main ${newtoponame} "./tunnels/"${file} case_id >> ${case}"_log.csv"
        done
        echo ${data_dir}/${Case}/
        cp ./${case}"_log.csv" ./${output_folder}/
        rm ./${case}"_log.csv"
        for file in $(ls ${work_dir})
        do
            name=${file%%.*}
            type=${name##*_}
            case=${name%_*}
            if [ $type = "topo" ]
            then
            rm ${case}"_topo.csv"
            rm ${case}${tunnel_name}
            fi
        done
    done
# done

echo ${num} > "num.csv"
}

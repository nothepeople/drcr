#!/bin/bash
{
work_dir=`pwd`
type_name="data/DelayRange"
data_dir=${work_dir}/${type_name}
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
if [ -f srlg_log ]
then
  rm srlg_log
fi
if [ -f link_log ]
then
  rm link_log
fi
if [ -f Result_Analysis ]
then
  rm Result_Analysis
fi
make main
make check_res
make split
num=0
mkdir "LagrangianKsp"
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
        newtunnelname=${type_name}"_"${Case}"flow_info.csv"
        cp -f $data_dir/${Case}/flow_info.csv $work_dir/$newtunnelname
        ./split $newtunnelname

        echo $work_dir/$newtunnelname
        echo "-----------------------------------------------------------"
        for file in $(ls ${work_dir}"/tunnels/")
        do
            name=${file%%.*}
            type=${name##*_}
            case=${name%_*}
            echo $file
            timeout 1000s ./main ${newtoponame} "./tunnels/"${file} >> ${case}"_log.csv"
            # echo ${case}"_log"
            num=$(($num + 1))
            a=`expr $num % 200`
            if [ $a = 0 ]
            then
                echo "hello"
                ./check_res srlg_log.csv link_log.csv
            fi
        done
        echo ${data_dir}/${Case}/
        cp ./${case}"_log.csv" ./LagrangianKsp/
        rm ./${case}"_log.csv"
        for file in $(ls ${work_dir})
        do
            name=${file%%.*}
            type=${name##*_}
            case=${name%_*}
            if [ $type = "topo" ]
            then
            rm ${case}"_topo.csv"
            rm ${case}"flow_info.csv"
            fi
        done
    done
# done

echo ${num} > "num.csv"
}

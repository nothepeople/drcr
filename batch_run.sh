#!/bin/bash
{
work_dir=`pwd`
type_name="SrlgDisjoint"
tunnel_name="large.csv"
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

make main
# g++ -o check_res check_res.cc
g++ -o split split_tunnel.cc
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
            timeout 1000s ./main ${newtoponame} "./tunnels/"${file} >> ${case}"_log.csv"
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
            rm ${case}${tunnel_name}
            fi
        done
    done
# done

echo ${num} > "num.csv"
}

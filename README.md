# robocap-tools for in/extrinsic calibration.

* download repo

```shell
git clone https://github.com/frodobots-org/robocap-tools.git
```

* initialize submodule

```shell
git submodule update --init --recursive
```

* build docker image

```shell
make
```

* download dataset(google drive)
  
  https://drive.google.com/drive/folders/1SEXwZtHckQblQ_WbQn9JkqGuP2ZRZf-I?dmr=1&ec=wgc-drive-globalnav-goto
  
  

* mount dataset and scripts while running the docker container
  
  unzip the dataset you downloaded.

```
docker run -it -v /home/dell/ethan/linux/robocap-tools/scripts:/robocap-scripts -v /home/dell/ethan/linux/dc0c9f0cd3efa0c6:/data robocap-calib bash
```

* run a camera intrinsic calibration program
  
  > cam0-cam5

```shell
/robocap-scripts/02_cams_intrinsic.sh /data/ cam0
```

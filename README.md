# theta_simple_stitching

### Target device
Ricoh Theta S

### Execute
1) Launch Theta in Live mode  
Press the power and camera buttons to start  
2) Change the authority  
`sudo chmod 666 /dev/bus/usb/001/*`  
3) `roslaunch theta_simple_stitching theta.launch`  
4) Launch rviz  
5) Select `/image/mercator/Image` to display the image

### Reference
[RICHO THETAの画像をpythonでスティッチング（改）](http://blog.livedoor.jp/tmako123-programming/archives/50769806.html)

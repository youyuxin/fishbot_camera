调整接口：

LED的亮度，0-255
http://172.20.10.12/control?var=led_intensity&val=0

图像质量： 4-63
http://172.20.10.12/control?var=quality&val=37

水平镜像：
http://172.20.10.12/control?var=hmirror&val=0

上下反转：
http://172.20.10.12/control?var=vflip&val=1

人脸检测：
http://172.20.10.12/control?var=face_detect&val=1

图像大小：
http://172.20.10.12/control?var=framesize&val=4

{  
  "framesize": [  
    {  
      "label": "UXGA(1600x1200)",  
      "value": 13  
    },  
    {  
      "label": "SXGA(1280x1024)",  
      "value": 12  
    },  
    {  
      "label": "HD(1280x720)",  
      "value": 11  
    },  
    {  
      "label": "XGA(1024x768)",  
      "value": 10  
    },  
    {  
      "label": "SVGA(800x600)",  
      "value": 9  
    },  
    {  
      "label": "VGA(640x480)",  
      "value": 8  
    },  
    {  
      "label": "HVGA(480x320)",  
      "value": 7  
    },  
    {  
      "label": "CIF(400x296)",  
      "value": 6  
    },  
    {  
      "label": "QVGA(320x240)",  
      "value": 5  
    },  
    {  
      "label": "240x240",  
      "value": 4  
    },  
    {  
      "label": "HQVGA(240x176)",  
      "value": 3  
    },  
    {  
      "label": "QCIF(176x144)",  
      "value": 2  
    },  
    {  
      "label": "QQVGA(160x120)",  
      "value": 1  
    },  
    {  
      "label": "96x96",  
      "value": 0  
    }  
  ]  
}


特殊效果：
http://172.20.10.12/control?var=special_effect&val=1
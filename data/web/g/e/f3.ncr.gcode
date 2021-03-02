(1009)
(Machine)
(  vendor: Autodesk)
(  description: Generic Cutting Machine)
N10 G90
N15 G21

(2D Profile2)
N20 M36 T1
(Through cutting)

N25 G0 X233.893 Y79.588
(TURN ON CUTTING)
N30 M7
(POINT-PIERCE)
N35 G1 Y81.588 F600
N40 G3 X232.893 Y82.588 I-1
N45 G1 X230.963
N50 Y64.625
N55 X246.707
N60 G2 X247.207 Y64.125 J-0.5
N65 G1 Y45.101
N70 G2 X246.707 Y44.601 I-0.5
N75 G1 X230.963
N80 Y23.612
N85 X249.548
N90 G2 X250.048 Y23.112 J-0.5
N95 G1 Y18.359
N100 X267.455
N105 Y82.588
N110 X252.575
N115 G2 X252.075 Y83.088 J0.5
N120 G1 Y87.359
N125 X248.319
N130 Y83.088
N135 G2 X247.819 Y82.588 I-0.5
N140 G1 X232.893
N145 G3 X231.893 Y81.588 J-1
N150 G1 Y79.588
(TURN OFF CUTTING)
N155 M8
N160 G0 X200.959 Y83.359
(TURN ON CUTTING)
N165 M7
(POINT-PIERCE)
N170 G1 X202.959 F600
N175 G3 X203.959 Y84.359 J1
N180 G1 Y87.359
N185 X193.866
N190 X184.168 Y57.941
N195 X195.935 Y18.359
N200 X203.959
N205 Y84.359
N210 G3 X202.959 Y85.359 I-1
N215 G1 X200.959
(TURN OFF CUTTING)
N220 M8
N225 G0 X166.081 Y81.956
(TURN ON CUTTING)
N230 M7
(POINT-PIERCE)
N235 G1 X164.081 F600
N240 G3 X163.081 Y80.956 J-1
N245 G1 Y67.685
N250 X168.93 Y87.359
N255 X163.081
N260 Y80.956
N265 G3 X164.081 Y79.956 I1
N270 G1 X166.081
(TURN OFF CUTTING)
N275 M8
N280 G0 X164.719 Y21.359
(TURN ON CUTTING)
N285 M7
(POINT-PIERCE)
N290 G1 Y19.359 F600
N295 G3 X165.719 Y18.359 I1
N300 G1 X168.829
N305 X163.081 Y41.823
N310 Y18.359
N315 X165.719
N320 G3 X166.719 Y19.359 J1
N325 G1 Y21.359
(TURN OFF CUTTING)
N330 M8
N335 G0 X131.873 Y32.516
(TURN ON CUTTING)
N340 M7
(POINT-PIERCE)
N345 G1 X129.874 Y32.484 F600
N350 G3 X128.89 Y31.468 I0.016 J-1
N355 G2 X128.079 Y18.359 I-53.799 J-3.25
N360 G1 X136.078
N365 Y87.359
N370 X126.11
N375 G2 X126.763 Y78.189 I-48.774 J-8.079
N380 G1 Y72.835
N385 G2 X126.263 Y72.335 I-0.5
N390 G1 X102.112
N395 G2 X101.612 Y72.835 J0.5
N400 G1 Y80.246
N405 G3 X100.017 Y87.29 I-7.346 J2.039
N410 G1 X99.976 Y87.32
N415 X99.936 Y87.348
N420 X99.92 Y87.359
N425 X96.268
N430 X96.017 Y87.116
N435 X95.777 Y86.863
N440 X95.548 Y86.6
N445 X95.331 Y86.327
N450 X95.126 Y86.044
N455 X94.935 Y85.753
N460 X94.756 Y85.453
N465 X94.59 Y85.146
N470 X94.439 Y84.832
N475 X94.301 Y84.511
N480 X94.178 Y84.184
N485 X94.07 Y83.853
N490 X93.976 Y83.517
N495 X93.897 Y83.177
N500 X93.833 Y82.834
N505 X93.785 Y82.488
N510 X93.752 Y82.141
N515 X93.734 Y81.793
N520 X93.732 Y81.444
N525 X93.745 Y81.095
N530 X93.774 Y80.748
N535 X93.818 Y80.402
N540 X93.878 Y80.058
N545 X93.952 Y79.717
N550 X94.042 Y79.38
N555 X94.146 Y79.047
N560 G3 X101.339 Y68.216 I10.321 J-0.95
N565 G2 X124.442 Y50.26 I-48.473 J-86.209
N570 X128.89 Y31.472 I-33.937 J-17.955
N575 G3 X129.906 Y30.484 I1 J0.012
N580 G1 X131.906 Y30.516
(TURN OFF CUTTING)
N585 M8
N590 G0 X99.944 Y31.866
(TURN ON CUTTING)
N595 M7
(POINT-PIERCE)
N600 G1 X101.828 Y32.538 F600
N605 G3 X102.433 Y33.816 I-0.336 J0.942
N610 X95.06 Y41.791 I-13.209 J-4.816
N615 G2 X75.337 Y56.181 I133.5 J203.697
N620 X68.834 Y80.241 I22.848 J19.085
N625 X69.289 Y87.359 I20.25 J2.279
N630 G1 X54.683
N635 X64.954 Y18.359
N640 X70.702
N645 G2 X69.308 Y32.373 I52.592 J12.309
N650 G1 Y38.925
N655 G2 X69.808 Y39.425 I0.5
N660 G1 X93.959
N665 G2 X94.459 Y38.925 J-0.5
N670 G1 Y26.757
N675 G3 X96.261 Y18.986 I9.938 J-1.79
N680 X102.864 Y23.909 I2.167 J3.984
N685 X102.433 Y33.816 I-21.804 J4.015
N690 X101.155 Y34.422 I-0.942 J-0.336
N695 G1 X99.272 Y33.749
(TURN OFF CUTTING)
N700 M8
N705 G0 X33.879 Y44.925
(TURN ON CUTTING)
N710 M7
(POINT-PIERCE)
N715 G1 X35.868 Y45.136 F600
N720 G3 X36.757 Y46.236 I-0.106 J0.994
N725 G1 X36.626 Y47.475
N730 X36.442 Y49.221
N735 X36.261 Y50.968
N740 X36.082 Y52.714
N745 X35.906 Y54.462
N750 X35.733 Y56.209
N755 X35.564 Y57.957
N760 X35.399 Y59.706
N765 X35.237 Y61.455
N770 X35.078 Y63.204
N775 X34.923 Y64.953
N780 X34.77 Y66.702
N785 X34.62 Y68.452
N790 X34.471 Y70.202
N795 X33.979 Y76.144
N800 G3 X29.715 Y39.301 I704.248 J-100.169
N805 G1 X37.503
N810 X36.757 Y46.236
N815 G3 X35.657 Y47.124 I-0.994 J-0.106
N820 G1 X33.668 Y46.913
(TURN OFF CUTTING)
N825 M8
N830 G0 X35.562 Y17.329
(TURN ON CUTTING)
N835 M7
(POINT-PIERCE)
N840 G1 X37.557 Y17.473 F600
N845 G3 X38.482 Y18.543 I-0.072 J0.997
N850 G1 X38.336 Y20.574
N855 X29.933
N860 X28.428 Y3.057
N865 G2 X27.93 Y2.6 I-0.498 J0.043
N870 G1 X1
N875 G2 X0.504 Y3.166 J0.5
N880 G1 X13.722 Y103.166
N885 G2 X14.218 Y103.6 I0.496 J-0.066
N890 G1 X51.834
N895 G2 X52.328 Y103.174 J-0.5
N900 G1 X53.045 Y98.359
N905 X74.772
N910 G2 X100.906 Y105.271 I20.608 J-25.058
N915 X120.986 Y98.359 I0.843 J-30.176
N920 G1 X136.078
N925 Y103.1
N930 G2 X136.578 Y103.6 I0.5
N935 G1 X162.581
N940 G2 X163.081 Y103.1 J-0.5
N945 G1 Y98.359
N950 X172.2
N955 X173.653 Y103.243
N960 G2 X174.132 Y103.6 I0.479 J-0.143
N965 G1 X198.529
N970 G2 X199.004 Y102.943 J-0.5
N975 G1 X197.492 Y98.359
N980 X203.959
N985 Y103.1
N990 G2 X204.459 Y103.6 I0.5
N995 G1 X247.819
N1000 G2 X248.319 Y103.1 J-0.5
N1005 G1 Y98.359
N1010 X252.075
N1015 Y103.1
N1020 G2 X252.575 Y103.6 I0.5
N1025 G1 X309.4
N1030 G2 X309.9 Y103.1 J-0.5
N1035 G1 Y83.088
N1040 G2 X309.4 Y82.588 I-0.5
N1045 G1 X294.459
N1050 Y3.1
N1055 G2 X293.959 Y2.6 I-0.5
N1060 G1 X267.955
N1065 G2 X267.455 Y3.1 J0.5
N1070 G1 Y7.359
N1075 X250.048
N1080 Y3.1
N1085 G2 X249.548 Y2.6 I-0.5
N1090 G1 X204.459
N1095 G2 X203.959 Y3.1 J0.5
N1100 G1 Y7.359
N1105 X199.205
N1110 X200.429 Y3.243
N1115 G2 X199.95 Y2.6 I-0.479 J-0.143
N1120 G1 X173.082
N1125 G2 X172.596 Y2.981 J0.5
N1130 G1 X171.524 Y7.359
N1135 X163.081
N1140 Y3.1
N1145 G2 X162.581 Y2.6 I-0.5
N1150 G1 X136.578
N1155 G2 X136.078 Y3.1 J0.5
N1160 G1 Y7.359
N1165 X121.987
N1170 G2 X77.178 Y7.358 I-22.404 J32.212
N1175 G1 X66.591 Y7.359
N1180 X67.214 Y3.174
N1185 G2 X66.72 Y2.6 I-0.495 J-0.074
N1190 G1 X40.098
N1195 G2 X39.599 Y3.064 J0.5
N1200 G1 X38.482 Y18.543
N1205 G3 X37.413 Y19.468 I-0.997 J-0.072
N1210 G1 X35.418 Y19.324
(TURN OFF CUTTING)
N1215 M8

N1220 M19
N1225 M30
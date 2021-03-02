(1001)
(Machine)
(  vendor: Autodesk)
(  description: Generic Cutting Machine)
N10 G90
N15 G21

(2D Profile1)
N20 M36 T1
(Through cutting)

N25 G0 X34.016 Y43.51
(TURN ON CUTTING)
N30 M7
(POINT-PIERCE)
N35 G1 X35.516 Y40.912 F600
N40 G3 X38.114 Y39.412 I2.598 J1.5
N45 G1 X56.312
N50 G2 X56.812 Y38.912 J-0.5
N55 X56.303 Y23.39 I-113.748 J-4.04
N60 G1 X64.206
N65 Y92.89
N70 G2 X64.706 Y93.39 I0.5
N75 G1 X84.274
N80 G2 X84.749 Y93.044 J-0.5
N85 G1 X96.893 Y55.533
N90 Y92.89
N95 G2 X97.393 Y93.39 I0.5
N100 G1 X116.96
N105 G2 X117.46 Y92.89 J-0.5
N110 G1 Y88.39
N115 X132.661
N120 X133.24 Y88.828
N125 X133.829 Y89.254
N130 X134.427 Y89.667
N135 X135.034 Y90.068
N140 X135.649 Y90.455
N145 X136.273 Y90.829
N150 X136.904 Y91.189
N155 X137.543 Y91.536
N160 X138.189 Y91.869
N165 X138.842 Y92.188
N170 X139.502 Y92.493
N175 X140.168 Y92.784
N180 X140.84 Y93.061
N185 X141.518 Y93.323
N190 X142.201 Y93.57
N195 X142.89 Y93.803
N200 X143.583 Y94.021
N205 X144.281 Y94.224
N210 X144.983 Y94.412
N215 X145.689 Y94.585
N220 X146.399 Y94.742
N225 X147.112 Y94.885
N230 X147.827 Y95.012
N235 X148.546 Y95.124
N240 X149.266 Y95.22
N245 X149.988 Y95.301
N250 X150.712 Y95.367
N255 X151.437 Y95.417
N260 X152.163 Y95.451
N265 X152.89 Y95.47
N270 X153.617 Y95.473
N275 X154.344 Y95.461
N280 X155.07 Y95.433
N285 X155.795 Y95.39
N290 X156.52 Y95.331
N295 X157.243 Y95.256
N300 X157.964 Y95.166
N305 X158.683 Y95.061
N310 X159.4 Y94.94
N315 X160.114 Y94.804
N320 X160.825 Y94.653
N325 X161.532 Y94.486
N330 X162.236 Y94.304
N335 X162.936 Y94.108
N340 X163.631 Y93.896
N345 X164.322 Y93.669
N350 X165.007 Y93.428
N355 X165.688 Y93.172
N360 X166.362 Y92.902
N365 X167.031 Y92.617
N370 X167.694 Y92.318
N375 X168.349 Y92.004
N380 X168.998 Y91.677
N385 X169.64 Y91.336
N390 X170.275 Y90.981
N395 X170.901 Y90.613
N400 X171.52 Y90.232
N405 X172.13 Y89.837
N410 X172.732 Y89.429
N415 X173.325 Y89.008
N420 X173.908 Y88.575
N425 X174.483 Y88.13
N430 X175.047 Y87.672
N435 X175.602 Y87.202
N440 X176.146 Y86.721
N445 X176.68 Y86.228
N450 G2 X181.489 Y64.563 I-35.911 J-19.337
N455 G1 X181.494 Y63.777
N460 X181.498 Y62.992
N465 X181.5 Y62.206
N470 Y53.644
N475 G2 X181 Y53.144 I-0.5
N480 G1 X157.597
N485 G2 X157.097 Y53.644 J0.5
N490 G1 Y69.32
N495 G3 X155.645 Y78.485 I-15.56 J2.232
N500 X150.027 Y74.211 I-2.089 J-3.084
N505 X149.591 Y68.601 I18.926 J-4.292
N510 G1 Y26.794
N515 G3 X151.189 Y17.318 I15.364 J-2.281
N520 X156.616 Y21.321 I2.051 J2.899
N525 X157.097 Y27.621 I-20.349 J4.721
N530 G1 Y38.912
N535 G2 X157.597 Y39.412 I0.5
N540 G1 X181
N545 G2 X181.5 Y38.912 J-0.5
N550 X178.691 Y13.841 I-62.58 J-5.682
N555 X152.2 Y0.832 I-22.775 J12.901
N560 X128.697 Y11.43 I-2.322 J26.211
N565 X125.188 Y34.852 I55.696 J20.319
N570 G1 Y61.148
N575 G2 X125.548 Y72.39 I121.8 J1.723
N580 G1 X117.46
N585 Y2.89
N590 G2 X116.96 Y2.39 I-0.5
N595 G1 X96.448
N600 G2 X95.969 Y2.747 J0.5
N605 G1 X84.774 Y40.371
N610 Y2.89
N615 G2 X84.274 Y2.39 I-0.5
N620 G1 X64.706
N625 G2 X64.206 Y2.89 J0.5
N630 G1 Y7.39
N635 X49.085
N640 G2 X6.046 Y8.479 I-20.846 J27.197
N645 X0.795 Y33.141 I33.672 J20.059
N650 G1 X0.504 Y34.014
N655 X0.5 Y34.852
N660 Y61.148
N665 G2 X3.417 Y82.861 I46.746 J4.773
N670 X30.506 Y94.911 I22.846 J-14.886
N675 X54.862 Y80.977 I1.8 J-25.107
N680 X56.812 Y62.206 I-68.302 J-16.58
N685 G1 Y53.644
N690 G2 X56.312 Y53.144 I-0.5
N695 G1 X32.909
N700 G2 X32.409 Y53.644 J0.5
N705 G1 Y69.32
N710 G3 X30.69 Y78.838 I-12.396 J2.676
N715 G1 X30.457 Y78.931
N720 X30.218 Y79.01
N725 X29.976 Y79.073
N730 X29.729 Y79.122
N735 X29.481 Y79.154
N740 X29.23 Y79.172
N745 X28.979 Y79.173
N750 X28.729 Y79.159
N755 X28.48 Y79.129
N760 X28.233 Y79.083
N765 X27.99 Y79.022
N770 X27.75 Y78.946
N775 X27.517 Y78.855
N780 X27.289 Y78.75
N785 X27.068 Y78.631
N790 X26.855 Y78.498
N795 X26.651 Y78.352
N800 X26.457 Y78.193
N805 X26.273 Y78.023
N810 X26.099 Y77.841
N815 X25.938 Y77.649
N820 X25.789 Y77.447
N825 X25.653 Y77.237
N830 X25.53 Y77.018
N835 X25.421 Y76.792
N840 X25.327 Y76.559
N845 X25.247 Y76.322
N850 X25.182 Y76.079
N855 X25.133 Y75.833
N860 X25.099 Y75.585
N865 X25.081 Y75.334
N870 X25.079 Y75.084
N875 X25.092 Y74.833
N880 X25.121 Y74.584
N885 X25.165 Y74.337
N890 X25.225 Y74.093
N895 X25.3 Y73.854
N900 G3 X24.903 Y68.602 I17.141 J-3.935
N905 G1 Y26.794
N910 G3 X26.478 Y17.166 I15.299 J-2.441
N915 G1 X26.691 Y17.05
N920 X26.911 Y16.947
N925 X27.138 Y16.859
N930 X27.37 Y16.785
N935 X27.606 Y16.727
N940 X27.845 Y16.684
N945 X28.086 Y16.656
N950 X28.329 Y16.644
N955 X28.572 Y16.647
N960 X28.815 Y16.667
N965 X29.055 Y16.701
N970 X29.293 Y16.751
N975 X29.527 Y16.817
N980 X29.757 Y16.897
N985 X29.981 Y16.992
N990 X30.198 Y17.101
N995 X30.408 Y17.223
N1000 X30.609 Y17.359
N1005 X30.802 Y17.508
N1010 X30.984 Y17.668
N1015 X31.156 Y17.84
N1020 X31.316 Y18.023
N1025 X31.464 Y18.216
N1030 X31.6 Y18.418
N1035 X31.722 Y18.628
N1040 X31.831 Y18.845
N1045 X31.925 Y19.069
N1050 X32.005 Y19.299
N1055 X32.07 Y19.533
N1060 X32.119 Y19.771
N1065 X32.153 Y20.012
N1070 X32.172 Y20.254
N1075 X32.175 Y20.497
N1080 X32.163 Y20.74
N1085 X32.134 Y20.981
N1090 X32.091 Y21.221
N1095 X32.032 Y21.456
N1100 X31.958 Y21.688
N1105 G3 X32.409 Y27.621 I-16.559 J4.241
N1110 G1 Y38.912
N1115 G2 X32.909 Y39.412 I0.5
N1120 G1 X38.114
N1125 G3 X40.712 Y40.912 J3
N1130 G1 X42.212 Y43.51
(TURN OFF CUTTING)
N1135 M8

N1140 M19
N1145 M30

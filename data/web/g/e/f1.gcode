
G90 G00 X0 Y0 F4200
G00 X5 Y5
G92

G00 X13 Y37
M03     (plasma start)
G42 R1.0
G01 X10 Y40
G02 X40 R35 F600
G01 X30 Y20
X20
X10 Y40
X14 Y38
G40
M05     (plasma stop)

G00 X-3 Y44 F3000

M03     (plasma start)
G41 R1.0
G01 X0 Y45 F600
X5 Y50 
X45
X50 Y45
Y35
X45 Y40
Y35
X50 Y30
Y20
X45 Y25
Y20
G02 Y10 R5
G03 X40 Y5 R5
G02 X30 R5
G03 X20 R5
G02 X10 R5
G03 X5 Y10 R5
G02 Y20 R5
G01 Y25
X0 Y20
Y30
X5 Y35
Y40
X0 Y35
Y45
X-3 Y48
G40
M05     (plasma stop)

G90 G00 X0 Y0 F4200

%
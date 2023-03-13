MODULE GridScan_T_ROB1
LOCAL VAR extjoint extj := [9E9,9E9,9E9,9E9,9E9,9E9];
LOCAL VAR confdata conf := [0,0,0,0];
PERS tooldata D455:=[TRUE,[[159.145,-14.956,419.61],[0.48851,-0.50586,0.51472,-0.49043]],[0.001,[159.145,-14.956,419.61],[1,0,0,0],0,0,0]];
LOCAL PERS wobjdata ScanFrame:=[FALSE,TRUE,"",[[-986.4,-3205.1,250.2],[0.99989,-0.00159,0.00286,0.0142]],[[0,0,0],[1,0,0,0]]];
LOCAL PERS speeddata Speed000:=[600,180,5000,1080];
LOCAL PERS num Wait000:=5;
PROC Main()
ConfL \Off;
MoveAbsJ [[-90,-0,0,0,0,0],extj],Speed000,fine,D455;
MoveJ [[200,200,600],[0,0,1,0],[-3,-2,0,1],extj],Speed000,fine,D455 \WObj:=ScanFrame;
WaitTime \InPos,Wait000;
MoveJ [[733.202,200,600],[0,0,1,0],[-3,-2,0,1],extj],Speed000,fine,D455 \WObj:=ScanFrame;
WaitTime \InPos,Wait000;
MoveJ [[1266.404,200,600],[0,0,1,0],[-2,0,-2,1],extj],Speed000,fine,D455 \WObj:=ScanFrame;
WaitTime \InPos,Wait000;
MoveJ [[1799.606,200,600],[0,0,1,0],[-2,0,-2,1],extj],Speed000,fine,D455 \WObj:=ScanFrame;
WaitTime \InPos,Wait000;
MoveJ [[2332.808,200,600],[0,0,1,0],[-2,0,-2,1],extj],Speed000,fine,D455 \WObj:=ScanFrame;
WaitTime \InPos,Wait000;
MoveJ [[200,613.628,600],[0,0,1,0],[-3,-2,0,1],extj],Speed000,fine,D455 \WObj:=ScanFrame;
WaitTime \InPos,Wait000;
MoveJ [[733.202,613.628,600],[0,0,1,0],[-3,-2,0,1],extj],Speed000,fine,D455 \WObj:=ScanFrame;
WaitTime \InPos,Wait000;
MoveJ [[1266.404,613.628,600],[0,0,1,0],[-2,0,-2,1],extj],Speed000,fine,D455 \WObj:=ScanFrame;
WaitTime \InPos,Wait000;
MoveJ [[1799.606,613.628,600],[0,0,1,0],[-2,0,-2,1],extj],Speed000,fine,D455 \WObj:=ScanFrame;
WaitTime \InPos,Wait000;
MoveJ [[2332.808,613.628,600],[0,0,1,0],[-2,0,-2,1],extj],Speed000,fine,D455 \WObj:=ScanFrame;
WaitTime \InPos,Wait000;
MoveJ [[200,1027.257,600],[0,0,1,0],[-3,-2,0,1],extj],Speed000,fine,D455 \WObj:=ScanFrame;
WaitTime \InPos,Wait000;
MoveJ [[733.202,1027.257,600],[0,0,1,0],[-3,-2,0,1],extj],Speed000,fine,D455 \WObj:=ScanFrame;
WaitTime \InPos,Wait000;
MoveJ [[1266.404,1027.257,600],[0,0,1,0],[-2,0,-2,1],extj],Speed000,fine,D455 \WObj:=ScanFrame;
WaitTime \InPos,Wait000;
MoveJ [[1799.606,1027.257,600],[0,0,1,0],[-2,0,-2,1],extj],Speed000,fine,D455 \WObj:=ScanFrame;
WaitTime \InPos,Wait000;
MoveJ [[2332.808,1027.257,600],[0,0,1,0],[-2,0,-2,1],extj],Speed000,fine,D455 \WObj:=ScanFrame;
WaitTime \InPos,Wait000;
MoveAbsJ [[-90,-0,0,0,0,0],extj],Speed000,fine,D455;
ENDPROC
ENDMODULE
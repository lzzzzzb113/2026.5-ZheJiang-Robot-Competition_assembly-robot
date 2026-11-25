.global SetPendSVPro
.type   SetPendSVPro, %function
SetPendSVPro:
    LDR R0, =0xE000ED20
    LDR R1, =0x00FF0000
    STR R1, [R0]
    BX LR

.global SetPendSV
.type   SetPendSV, %function
SetPendSV:
    LDR R0, =0xE000ED04
    LDR R2, =(0x1 << 28)
    LDR R1, [R0]
    ORR R1, R1, R2
    STR R1, [R0]
    BX LR

.global CLRPendSV
.type   CLRPendSV, %function
CLRPendSV:
    LDR R0, =0xE000ED04
    LDR R2, =(0x1 << 27)
    LDR R1, [R0]
    ORR R1, R1, R2
    STR R1, [R0]
    BX LR
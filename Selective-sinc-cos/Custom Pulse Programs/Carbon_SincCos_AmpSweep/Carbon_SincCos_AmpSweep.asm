
;***************************************************************************
      nolist
      include  'ioequ.asm'
      include  'FPGA.asm'
      list
;***************************************************************************

                org     x:$0


PARAM_BASE      equ     *       ; Memory for pulse program parameters
RXG1            ds      1       ; 00 - Receiver gain block 1
RXG2            ds      1       ; 01 - Receiver gain block 2
DEC1            ds      1       ; 02 - Decimation for CIC1
DEC5            ds      1       ; 03 - Decimation for CIC5
DECFIR          ds      1       ; 04 - Decimation for FIR
ATT1            ds      1       ; 05 - Attenuation for CIC1
DELAYFIR        ds      1       ; 06 - Delay for CIC5
ATTFIR          ds      1       ; 07 - Attenuation for FIR
Ntaps           ds      1       ; 08 - Taps for FIR
TXF00           ds      1       ; 09 - Tx Frequency word 0
TXF01           ds      1       ; 10 - Tx Frequency word 1
RXSETCHANNEL    ds      1       ; 11 - RxAmp set channel code
RXSETGAIN       ds      1       ; 12 - RxAmp set gain code
RXF00           ds      1       ; 13 - Rx Frequency word 0
RXF01           ds      1       ; 14 - Rx Frequency word 1
                ds      1       ; 15 - Rx Frequency word 2
                ds      1       ; 16 - Rx Frequency word 3
RXP0            ds      1       ; 17 - Rx Phase word 0
NRSCANS         ds      1       ; 18 - Number of scans to perform
EXPDELAY        ds      1       ; 19 - Delay between experiments
PGO             ds      1       ; 20 - Pulse gate overhead delay
GRADRESET       ds      1       ; 21 - 1 if gradients are to be reset
LFRXAMP         ds      1       ; 22 - 1 if low frequency Kea
SKIPPNTS        ds      1       ; 23 - Points to skip at start of acquisition
JITTER_CH1      ds      1       ; 24 - DDS channel 1 antiphase jitter parameter
JITTER_CH2      ds      1       ; 25 - DDS channel 2 antiphase jitter parameter
SoftVersion     ds      1       ; 26 - FPGA software version return
UseTrigger      ds      1       ; 27 - Use the trigger input (0/1)

; Pulse program info

TABLE1          ds      1       ; 28 - Table 1
TABLE2          ds      1       ; 29 - Table 2
TXP1            ds      1       ; 30 - Tx phase 1
NR1             ds      1       ; 31 - Number 1
DELAY1          ds      1       ; 32 - Delay 1
NRDataPnts             ds      1       ; 33 - Number DataPnts



;***************************************************************************
;
;       Space for filter coeffs, samples etc
;
        org    y:$0

CONSTANTS     equ     *     
TTL           ds      1                ; TTL state
DATA_ADRS     ds      1                ; Acquisition memory address for append mode
TX_AMP        ds      1                ; Current Tx amplitude
TX_PHASE      ds      1                ; Current Tx phase
TMP           ds      1                ; temporary variables

; **************************************************************************
; 
        org     p:$002000               ; start

        move    r0,x:(r6)+              ; Save registers
        move    r1,x:(r6)+
        move    r2,x:(r6)+
        move    r3,x:(r6)+
        move    r4,x:(r6)+
        move    r5,x:(r6)+
        move    r7,x:(r6)+
        move    x0,x:(r6)+
        move    x1,x:(r6)+
        move    y0,x:(r6)+
        move    y1,x:(r6)+
        move    b0,x:(r6)+
        move    b1,x:(r6)+
        move    b2,x:(r6)+

         

        movep   #$0FFFE1,x:A_BCR        ; Set up wait states, 7 for AA3
      ;  movep   #$0F9FE1,x:A_BCR        ; Set up wait states, 4 for AA3

      ;  movep   #$030421,x:A_AAR0       ; Map SRAM memory start to 0x30000

;Startup sequence
        move    #$000000,a1
        move    a1,x:FPGA_TTL
        move    #$000000,a1                     ; turn off reset pulse
        move    a1,x:FPGA_Sync
; AD9910
        move    #$000000,a1
        move    a1,x:FPGA_DDS_CR
; ADC LTC2207
        move    #$00d6d8,a1
        move    a1,x:FPGA_ADC_SW            ; startup time from shutdown
        move    #$000003,a1                 ; 3 for dithering 1 without
        move    a1,x:FPGA_ADC_CR            ; Start-up ADC
; Reset Sequence
; AD9910 and DDC
        move    #$000023,a1                     ; Reset AD9910 and CIC and DDC-DDS
        move    a1,x:FPGA_Sync
        move    #$000000,a1                     ; turn off reset pulse
        move    a1,x:FPGA_Sync
; Setup DDC
        move    #$000001,a1                 ; clear on set 0
        move    a1,x:FPGA_DRP1_CR
; Set DDC DDS Frequency
        move    x:RXF00,a1
        move    a1,x:FPGA_DRP1_PI          ; Update DRP DDS frequency
        move    x:RXF01,a1
        move    a1,x:FPGA_DRP1_PI
; Set DDC DDS Phase
        move    x:RXP0,a1
        move    a1,x:FPGA_DRP1_PO          ; Update DRP DDS phase
        move    #0,a1
        move    a1,x:FPGA_DRP1_PO
; Set CIC Fast Mode Specs
        move    #>1,a1
        move    a1,x:FPGA_DRP1_FASTMODE
        move    #>65,a1                     ; Minimum value for largest dwell times
        move    a1,x:FPGA_DRP1_MAXSpeedR
        move    x:SKIPPNTS,a1               ; Number of points to ignore from 
        move    a1,x:FPGA_DRP1_SD           ; Start if data collected
; Set CIC Decimation and scale value
        move    x:DEC1,a                    ; Update CIC Decimation
        move    a1,x:FPGA_DRP1_Dec
        move    x:ATT1,a                    ; Update Scale
        move    a1,x:FPGA_DRP1_Sca
        move    #$000001,a1                 ; turn off reset pulse
        move    a1,x:FPGA_Sync
; Set up filter coeffs
        move    #$000040,a1
        move    a1,x:FPGA_TTL
        move    #$10,r1                     ; Coefficients are stored from y:#$10
        move    #$000003,a1
        move    a1,x:FPGA_DRP1_FIRC
        move    #$000001,a1
        move    a1,x:FPGA_DRP1_FIRC
        do      x:Ntaps,fcl                  ; Do Ntaps coeffs
        move    y:(r1)+,a1
        move    a1,x:FPGA_DRP1_FIRData       ; Write bottom byte to data register 0 and inc pointer
fcl     nop
; Set FIR values
        move    x:DECFIR,a1                  ; Update CIC decimation set to 1
        move    a1,x:FPGA_DRP1_FIRC
        move    x:ATTFIR,a1
        move    a1,x:FPGA_DRP1_FIRScale
        move    #6000,a1
        move    a1,x:FPGA_DRP1_FIRRC
        move    x:DELAYFIR,a1
        move    #20,a1
        move    a1,x:FPGA_DRP1_FIRSD    ; Update filter delay count
        move    #$000040,a1
        move    a1,x:FPGA_TTL
; SETUP ADC
        move    #$000009,a1
        move    a1,x:FPGA_ADC_RW            ; delay from ADC capture to ND DDC
        move    #$000004,a1
        move    a1,x:FPGA_ADC_IR            ; Reset Overflow enable flag
        move    #$000000,a1
        move    a1,x:FPGA_ADC_IR
        move    #$000001,a1                 ; Reset Overflow bit
        move    a1,x:FPGA_ADC_OV
; Reset FIFO
        move    #$000001,a1                 ; RESET FIFO
        move    a1,x:FPGA_FIFO_CR
; Set up the AD9910 DDS

; Reset Serial control (new dual DDS board)
        move    #$001000,a1                 ; Reset serial IO to AD9910
        move    a1,x:FPGA_DDS_CR
        move    #$000400,a1                 ;
        move    a1,x:FPGA_DDS_CR
; Setup DDS PLL clk frequency
        move    #$000538,a1                 ; CFR3 32-16     #$0438
        move    a1,x:FPGA_DDS1_CFR3
        move    #$004128,a1                 ; CFR3 15-0      #$4120      1 GHz clock
        move    a1,x:FPGA_DDS1_CFR3
        move    #2,r7
        bsr     wait
        move    #$000538,a1                 ; CFR3 32-16     #$0438
        move    a1,x:FPGA_DDS2_CFR3
        move    #$004128,a1                 ; CFR3 15-0      #$4140    1 GHz clock
        move    a1,x:FPGA_DDS2_CFR3
        move    #2,r7
        bsr     wait
        move    #$000010,a1                 ; 000010 I/O update
        move    a1,x:FPGA_Sync
        move    #2,r7
        bsr     wait
        move    #$000000,a1                 ; 000010 I/O update
        move    a1,x:FPGA_Sync
        move    #100,r7
        bsr     wait

; Set Auxilary DAC value
        move    #$000000,a1                 ; Aux DAC 32-16
        move    a1,x:FPGA_DDS1_ADAC
        move    #$000085,a1                 ; Aux DAC 15-0
        move    a1,x:FPGA_DDS1_ADAC
        move    #2,r7
        bsr     wait
        move    #$000000,a1                 ; Aux DAC 32-16
        move    a1,x:FPGA_DDS2_ADAC
        move    #$000085,a1                 ; Aux DAC 15-0
        move    a1,x:FPGA_DDS2_ADAC
        move    #2,r7
        bsr     wait
; setup CFR1
        move    #$000049,a1                 ; CFR1 32-16
        move    a1,x:FPGA_DDS2_CFR1
        move    #$000000,a1                 ; CFR1 15-0
        move    a1,x:FPGA_DDS2_CFR1
        move    #2,r7
        bsr     wait
        move    #$000049,a1                 ; CFR1 32-16
        move    a1,x:FPGA_DDS1_CFR1
        move    #$000000,a1                 ; CFR1 15-0
        move    a1,x:FPGA_DDS1_CFR1
        move    #2,r7
        bsr     wait
; Switch off parallel update mode
        move    #$000000,a1        
        move    a1,x:FPGA_DDS1_PAR_MODE

; Setup CFR2
        move    #$000100,a1                 ; CFR2 32-16
        move    a1,x:FPGA_DDS1_CFR2
        move    #$000080,a1                 ; CFR2 15-0      parallel port mode off match latency on
        move    a1,x:FPGA_DDS1_CFR2
        move    #1,r7
        bsr     wait
        move    #$000100,a1                 ; CFR2 32-16
        move    a1,x:FPGA_DDS2_CFR2
        move    #$000080,a1                 ; CFR2 15-0      parallel port mode off match latency on
        move    a1,x:FPGA_DDS2_CFR2
        move    #1,r7
        bsr     wait
; Setup MultiChip sync
        move    #$002800,a1                 ; MSR 32-16 2c
        move    a1,x:FPGA_DDS1_MSR
        move    x:JITTER_CH1,a1
        move    a1,x:FPGA_DDS1_MSR
        move    #1,r7
        bsr     wait
        move    #$002800,a1                 ; MSR 32-16
        move    a1,x:FPGA_DDS2_MSR
        move    x:JITTER_CH2,a1
        move    a1,x:FPGA_DDS2_MSR
        move    #1,r7
        bsr     wait
; Auto generate iou
        move    #$000400,a1                 ;#$000400
        move    a1,x:FPGA_DDS_CR
        move    #2500,r7                   ; wait long time for loop
        bsr     wait

; Set values for AD9910 channel 1 profile 0
        move    #$000000,a1                 ;Profile 0   #003fff
        move    a1,x:FPGA_DDS1_Pro0
        move    #$000000,a1
        move    a1,x:FPGA_DDS1_Pro0
        move    x:TXF00,a1                  ; Output W1 to 9910 (freq byte 1)
        move    a1,x:FPGA_DDS1_Pro0
        move    x:TXF01,a1                  ; Output W2 to 9910 (freq byte 0)
        move    a1,x:FPGA_DDS1_Pro0
        move    #2,r7
        bsr     wait
; Set values for AD9910 channel 2 profile 0
        move    #$000000,a1                 ;Profile 0   #003fff
        move    a1,x:FPGA_DDS2_Pro0
        move    #$000000,a1
        move    a1,x:FPGA_DDS2_Pro0
        move    x:TXF00,a1                  ; Output W1 to 9910 (freq byte 1)
        move    a1,x:FPGA_DDS2_Pro0
        move    x:TXF01,a1                  ; Output W2 to 9910 (freq byte 0)
        move    a1,x:FPGA_DDS2_Pro0
        move    #2,r7
        bsr     wait
; Set DDC DDS Frequency
        move    x:RXF00,a1
        move    a1,x:FPGA_DRP1_PI          ; Update DRP DDS frequency
        move    x:RXF01,a1
        move    a1,x:FPGA_DRP1_PI
; Clear Accumulator and setup CFR1
        move    #$000049,a1                 ; CFR1 32-16
        move    a1,x:FPGA_DDS1_CFR1
        move    #$002000,a1                 ; CFR1 15-0
        move    a1,x:FPGA_DDS1_CFR1
        move    #2,r7
        bsr     wait
        move    #$000049,a1                 ; CFR1 32-16
        move    a1,x:FPGA_DDS2_CFR1
        move    #$002000,a1                 ; CFR1 15-0
        move    a1,x:FPGA_DDS2_CFR1
        move    #2,r7
        bsr     wait
; Sync AD9910 and internal DDC
        move    #$000008,a1                 ; SYNC pulse
        move    a1,x:FPGA_Sync
        move    #$000002,a1                 ; Clear Sync port
        move    a1,x:FPGA_Sync
        move    #$000004,a1                 ; Start CE DDC-DDS
        move    a1,x:FPGA_Sync
        move    #$000010,a1                 ; I/O Update to clear registers
        move    a1,x:FPGA_Sync
        move    #$000000,a1                 ; Clear Sync port
        move    a1,x:FPGA_Sync
        move    #2,r7
        bsr     wait

        move    #$000049,a1  ; CFR1 32-16
        move    a1,x:FPGA_DDS1_CFR1
        move    #$000000,a1  ; CFR1 15-0
        move    a1,x:FPGA_DDS1_CFR1
        move    #100,r7
        bsr     wait
        move    #$000049,a1  ; CFR1 32-16
        move    a1,x:FPGA_DDS2_CFR1
        move    #$000000,a1  ; CFR1 15-0
        move    a1,x:FPGA_DDS2_CFR1
        move    #100,r7
        bsr     wait

        nop
        nop
        nop
        nop
;  Configure serial ports

        movep   #$2C,x:A_PCRD           ; Set up SSI 1 on Port D
        movep   #$100803,x:A_CRA1       ; SSI 1 ctrl reg A /2 clk, 16 bit word transferred
        movep   #$13C3C,x:A_CRB1        ; SSI 1 ctrl reg B enable SSI port with sc1/2 are outputs

        movep   #$0007,x:A_PDRE         ; Select unused serial port
        movep   #$3c3c,x:A_CRB0         ; Disable SSI 0
        movep   #$24,x:A_PCRC

;
;***************************************************************************
; Select Receiver Amplifier 
        movep   #$2C,x:A_PCRC           ; Set up SSI 0
        movep   #$10080A,x:A_CRA0       ; /2 clk, 16 bit word transferred
        movep   #$13C3C,x:A_CRB0        ; Enable SSI port with sc1/2 are outputs
        move    x:RXSETCHANNEL,a1
        movep   #$0003,x:A_PDRE         ; Select first gain block
        move    a1,x:A_TX00
        move    #10,r7                  ; Wait 10 us
        bsr     wait
        movep   #$0007,x:A_PDRE         ; Select unused serial port
        movep   #$3c3c,x:A_CRB0         ; Disable SSI 0
        movep   #$24,x:A_PCRC
;
;***************************************************************************
; Set Rx gain
        movep   #$2C,x:A_PCRC           ; Set up SSI 0
        movep   #$10080A,x:A_CRA0       ; /10 clk, 16 bit word transferred
        movep   #$13C3C,x:A_CRB0        ; Enable SSI port with sc1/2 are outputs
        move    x:RXSETGAIN,a1
        lsr     #8,a
        move    a1,x:A_PDRE             ; Select first gain block
        move    x:RXG1,a1
        move    a1,x:A_TX00
        move    #10,r7                  ; Wait 10 us
        bsr     wait
        move    x:RXSETGAIN,a1
        and     #$0000FF,a
        move    a1,x:A_PDRE             ; Select second gain block
        move    x:RXG2,a1
        move    a1,x:A_TX00
        move    #10,r7                  ; Wait 10 us
        bsr     wait
        movep   #$0007,x:A_PDRE         ; Select unused serial port
        movep   #$3c3c,x:A_CRB0         ; Disable SSI 0
        movep   #$24,x:A_PCRC

; Intialise Lower 16 bits value
        movep   #$04,x:A_PCRD           ; Turn off SSI 1 on Port D (prevents serial noise)
;        movep   #$24,x:A_PCRD           ; Turn off SSI 1 on Port D (prevents serial noise)

; Intialise TTL value
        move    #$8000,a1   
        move    a1,y:TTL 
        move    a1,x:FPGA_TTL

; Initialise RF amplitude and phase
        move    #0,a1
        move    a1,y:TX_AMP  
        move    a1,y:TX_PHASE  

; Intialise Data address
        move    #$10000,a1
        move    a1,y:DATA_ADRS         
        move    a1,r5 

; Wait a bit   
        movep   #0,x:A_TLR2             ; Set up event timer
        move    #100,r3
        movep   r3,x:A_TCPR2            ; Set for first event
        movep   #$A01,x:A_TCSR2
        movep   #$200A00,x:A_TCSR2      ; Turn off timer





;
;***************************************************************************
; Delay
        move    #499999,a1
        sub     #9,a
        move    a1,r3
        movep   r3,x:A_TCPR2
        movep   #$200A01,x:A_TCSR2
        nop
        jclr    #21,x:A_TCSR2,*         ; Wait for pulse to end
        movep   #$200A00,x:A_TCSR2      ; Turn off timer

;
;***************************************************************************
; Generate a modulated external pulse (ch 2)
;
        clr a                           ; Clear the accumulator
        move    y:TTL,x1                ; Load the current TTL level
        move    #$00300,a1              ; Switch on rf bias (ch2)
        or      x1,a1                   ; Combine with ttl output
        move    a1,x:FPGA_TTL
; Start a timer to give pgo delay before RF comes on
        move    x:PGO,a1                ; All delays add to 1us before pulse comes on
        add     #5,a
        movep   a1,x:A_TCPR2
        nop
        movep   #$200A01,x:A_TCSR2
; Get the amplitude and phase table pointers
        move    x:TABLE1,r5
        move    x:TABLE2,r4
        move    x:NR1,r2
        move    r2,a0
        dec     a                        ; Decrement because first value already used
        move    a0,r2
; Set the rf output to its initial value
        move    y:(r5)+,a1               ; Load amplitude
        move    a1,x:FPGA_DDS2_Pro0
        move    y:(r4)+,a1               ; Load phase
        move    x:TXP1,y1
        add     y1,a                    ; Add phase to table
        move    a1,x:FPGA_DDS2_Pro0
; Wait for pgo delay to end
        jclr    #21,x:A_TCSR2,*
        movep   #$200A00,x:A_TCSR2      ; Turn off timer
; Start modulated pulse
        move    #$00302,a1              ; Switch on rf (external ch2)
        or      x1,a1                   ; and combine with internal ttl bias line
        move    a1,x:FPGA_TTL
        move    a,b
; Load step length
        move    x:DELAY1,a1
        lsl     #1,a
        sub     #198,a
        move    a1,a0
; Delay for correct first step length
        rep     a0
        nop
        rep     #60
        nop
; Calculate subsequent step length delay
        add     #149,a
        move    a1,a0
        do      r2,LBL1               ; Step the amplitude r2 times
        move    y:(r5)+,a1               ; Load amplitude
        move    a1,x:FPGA_DDS2_Pro0      ; Set amplitude
        move    y:(r4)+,a1               ; Load phase
        move    x:TXP1,y1
        add     y1,a                    ; Add phase to table
        move    a1,x:FPGA_DDS2_Pro0      ; Set phase
        rep     #17
        nop
; Adjust for correct step length
        rep     a0
        nop
LBL1  nop
; End Delay (correct for last pulse)
        rep     #98
        nop
; End pulse
        move    #$000000,a1
        or      x1,a1                   ; Combine with TTL output
        move    a1,x:FPGA_TTL
        move    #$000000,a1             ; Zero amplitude
        move    a1,x:FPGA_DDS2_Pro0
        move    a1,x:FPGA_DDS2_Pro0      ; Zero phase
;
;***************************************************************************
; Delay
        move    #499,a1
        sub     #9,a
        move    a1,r3
        movep   r3,x:A_TCPR2
        movep   #$200A01,x:A_TCSR2
        nop
        jclr    #21,x:A_TCSR2,*         ; Wait for pulse to end
        movep   #$200A00,x:A_TCSR2      ; Turn off timer

;
;***************************************************************************
; Acquire data (overwrite without delay)
        move    x:NRDataPnts,a1
        move    a1,x:FPGA_DRP1_SampleNo  ; FPGA for the wrap-up algorithm to use ### make sure this is the same number as the loop count or major errors could occur
        move    y:TTL,a1                ; Load the current TTL state
        or      #$000010,a              ; Reset CIC
        move    a1,x:FPGA_TTL           ; Send to FPGA
        and     #$ffffef,a              ; Remove CIC flag
        or      #$000001,a              ; Start ADC capture
        move    a1,x:FPGA_TTL           ; Send to FPGA
        movep   #0,x:A_TLR1             ; Set up ADC timer
        movep   #0,x:A_TCSR1            ; Disable timer
        movep   #$361,x:A_TCSR1         ; Set up timer1 (T1) for input capture
        move    #$10000,r5               ; Specify the save address
        move    x:NRDataPnts,r7
        do      r7,LBL2                ; Collect n samples
        jclr    #21,x:A_TCSR1,*         ; Wait for timer1 flag
        movep   #$200361,x:A_TCSR1      ; Clear timer1 flag
        move    x:FPGA_SampleA,a1        ; Load data from channel A
        nop
        nop
        move    a1,y:(r5)+              ; Save to memory
        move    x:FPGA_SampleB,a1       ; Load data from channel B
        nop
        nop
         move    a1,y:(r5)+              ; Save to memory
LBL2    nop
        move    y:TTL,a1                ; Stop ADC capture
        move    a1,x:FPGA_TTL           ; Send to FPGA


;*************************************************************
;        End pulse program
;
        jmp     SKIP1 
ABORT   move    #$10000,r5             ; Write 100 into the first 1000 memory location
        move    b1,y:(r5)+
        move    #$99,r7
        do      r7,ABORTLP
        move   #100,a1
        move    a1,y:(r5)+
ABORTLP nop

SKIP1    move    #$000000,a1
        move    a1,x:FPGA_TTL
        move    #$000000,a1
        move    a1,x:FPGA_DRP1_CR
        move    #$000000,a1             ; Adds delay between bus writes
        move    a1,x:FPGA_ADC_CR
        move    #$000100,a1
        move    a1,x:FPGA_DDS_CR
        nop
        move    x:FPGA_SoftBuild,a1
        move    a1,x:SoftVersion

SKIP2   move    x:-(r6),b2              ; Restore registers
        move    x:-(r6),b1
        move    x:-(r6),b0
        move    x:-(r6),y1
        move    x:-(r6),y0
        move    x:-(r6),x1
        move    x:-(r6),x0
        move    x:-(r6),r7
        move    x:-(r6),r5
        move    x:-(r6),r4
        move    x:-(r6),r3
        move    x:-(r6),r2
        move    x:-(r6),r1
        move    x:-(r6),r0
        rts

; 
;*************************************************************
;        Wait n x 1us routine (100MHz clock)
;        On entry, r7 contains "n"
; 
wait    rep     #90
        nop
        move    (r7)-
        move    r7,b
        tst     b
        bne     wait
        rts
; 
;*************************************************************
;        short fixed wait 100ns routine (100MHz clock)
; 
swait   rep     #4
        nop
        rts

; 
;*************************************************************
;        Short variable wait of n x 100 ns routine (100MHz clock)
;        On entry, r7 contains "n"
; 
svwait  move    (r7)-
        move    r7,b
        tst     b
        nop
        nop
        nop
        bne     svwait
        rts

; 
; *************************************************************
; 
;        TTL parameters
; 
ttl   dc      $00
; 
;*************************************************************

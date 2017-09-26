extern free
extern malloc

section .rodata
divMask: dd -0.5, -0.5, -0.5, -0.5
selectMask: dd 0x0000, 0x11111111, 0x11111111, 0x0000   


section .text

;typedef struct fluid_solver_t {
;    uint32_t N;       0          
;    float dt,          4   
;    float difff,       8  
;    float visc;        12       
;    float * u,         16 
     ;float* v,         24
     ;float * u_prev,
     ;float * v_prev; //velocity fields
;    float * dens, * dens_prev;  //density fields
;} __attribute__((__packed__)) fluid_solver;

%define offset_solver_n 0
%define offset_solver_u 16
%define offset_solver_v 24

global solver_set_bnd2
solver_set_bnd2:
    ;xor r12, r12
    ;mov r13, 

    .code:
        ;mov rbx, rdx
        add rbx, 4
        
        mov r12, rbx
        add r12, offset_solver_n + 4 ; el 4 funca aca?, de donde saco N? 
        add r12, 8
    .loop:
        movdqu xmm1, [r12]
        movdqu [rbx], xmm1

        sub r13, 4
        cmp r13, 0
        jle .fillBorders

        add rbx, 16
        add r12, 16
        jmp .loop

    .fillBorders:
        ;[0,0]
        mov rcx, rdx
        mov rbx, rdx
        add rbx, 4
        movss xmm2, [rbx]
        addss xmm1, xmm2
        ;mulss xmm1, 0.5f
        movss [rbx], xmm1
        add rdx, 4 * (offset_solver_n + 1) ; que onda las operaciones acá? y el N?

        ;[0, N+1]
        mov rbx, rdx
        sub rbx, 4
        movss xmm1, [rbx]
        ;add rbx, 4*(N+3)
        movss xmm2, [rbx]
        addss xmm1, xmm2
        ;mulss xmm1, 0.5f
        movss [rdx], xmm1

        ;[N+1, 0]
        mov rdx, rcx
        add rdx, (offset_solver_n + 2)*(offset_solver_n + 1) * 4
        mov rbx, rdx
        add rbx, 4
        movss xmm1, [rbx]
        sub rbx, (4 * (offset_solver_n + 3))
        movss xmm2, [rbx]
        addss xmm1, xmm2
        ;mulss xmm1, 0.5f

        ; [N+1, N+1]
        movss [rdx], xmm1
        add rdx, 4 * (offset_solver_n + 1)
        mov rbx, rdx
        sub rbx, 4
        movss xmm1, [rbx]
        sub rbx, 4 * (offset_solver_n + 1)
        movss xmm2, [rbx]
        addss xmm1, xmm2
        ;mulss xmm1, 0.5f
        movss [rbx], xmm1 

global solver_project_first
solver_project_first:
    ;solver_project_first ( solver, p, div  );
    ; RDI , RSI, RDX
    push rbp
    mov rbp, rsp
    push rbx
    push r12
    push r13
    push r14
    push r15

    xor R8, R8
    mov R8d , [ RDI ] ; Guardo N
    mov R9, R8; R9 = N
    add R9, 2; R9 = N+2

    mov R12, R8;
    shr R12, 1 ; Lo divido por dos que van a ser los movimientos que hay que hacer por fila

    mov RCX, R8; Le muevo N y le resto uno ya que la primera la salteo
    dec RCX
    .loop:
    cmp RCX, 0;
    ;jmp .fin:

    ; Obtengo y acomodo div
    mov RBX , RDX
    add RBX , R9 ; Lo pongo en la segunda fila

    ; Obtengo p
    ; Lo tengo en RSI
    add RSI , R9 ; Lo pongo en la segunda fila
    pxor XMM7, XMM7 ; Lo utilizare para poner 0s

    ; Obtener U
    lea R13 , [ RDI + offset_solver_u ] 
    ;Hay que sumarle uno, no? Lo mismo para el de V.

    ; Obtener V 
    lea R14 , [ RDI + offset_solver_v ]
    mov R15 , R14

    ; Los tres apuntan al inicio de sus respectivas matrices 
    add R13 , R9 ; El de U debe apuntar a la primer fila no borde
    add R15 , R9
    add R15 , R9 ; R14 queda abajo. R15 hay que hacerlo ir a la tercera

    pxor XMM0, XMM0    ; XMM0 = [  0 |  0 |  0 |  0 ]
    pxor XMM5, XMM5  

    movdqu XMM1 ,[R13] ; XMM1 = [ u3 | u2 | u1 | u0 ]
    movdqu XMM2 , XMM1 ; XMM2 = [ u3 | u2 | u1 | u0 ]

    pslldq XMM1 , 4    ; XMM1 = [ u2 | u1 | u0 | 0  ]
    psrldq XMM2 , 4    ; XMM2 = [ 0  | u3 | u2 | u1 ]

    addpd XMM0 , XMM1  ; XMM0 = [ 0  | u3    | u2      |  u1  ]
    subps XMM0 , XMM2  ; XMM0 = [ -  | u3-u1 | u2 - u0 |   -  ]

    movdqu XMM3 ,[R14] ; XMM3 = [ v03 | v02 | v01 | v00 ]
    movdqu XMM4 ,[R15] ; XMM4 = [ v13 | v12 | v11 | v10 ]

    subps XMM0 , XMM3
    addps XMM0 , XMM4 ; XMM0 = [ - | u3-u1 + v12 - v02 | u2 - u0 + v11 - v01 | - ]

    ;mov dw XMM5, R8 ; Mascara - XMM5 = [ 0 | 0 | 0 | n ]
    pslldq XMM5 , 4 ;  XMM5 = [ 0 | 0 | n | 0 ]
    movq XMM5, R8 ; Mascara - XMM5 = [ 0 | 0 | n | n ]
    movq XMM6, XMM5
    pslldq XMM6 , 8 ;  XMM6 = [ n | n | 0 | 0 ]
    paddd XMM5, XMM6;  XMM5 = [ n | n | n | n ]

    divps XMM0, XMM5  ; XMM0 = [ - | (u3-u1 + v12 - v02) / n | (u2 - u0 + v11 - v01) / n | - ]

    movdqu XMM5, [divMask]  ; Mascara - XMM5 = [ -0.5 | -0.5 | -0.5 | -0.5 ]
    mulps XMM0, XMM5  ; XMM0 = [ - | -0.5 * ( (u3-u1 + v12 - v02) / n ) | -0.5 *  ( (u2 - u0 + v11 - v01) / n ) | - ]

    movdqu XMM5, [selectMask] ; Mascara - XMM5 = [ 0 | 1 | 1 | 0 ]
    pand  XMM0, XMM5  ; XMM0 = [ 0 | -0.5 * ( (u3-u1 + v12 - v02) / n ) | -0.5 *  ( (u2 - u0 + v11 - v01) / n ) | 0 ]

    movdqu XMM1, [RBX] ; Agarro los valores que tenia en V para no pisar los que quiero (el primero) XMM1 = [ v3 | v2 | v1 | v0 ]
    pandn XMM1, XMM0  ; XMM1 = [ v3 | 0 | 0 | v0 ]
    addpd XMM0, XMM1  ; XMM0 = [ v3 | -0.5 * ( (u3-u1 + v12 - v02) / n ) | -0.5 *  ( (u2 - u0 + v11 - v01) / n ) | v0 ]

    movdqu [RBX] , XMM0 ; Muevo los valores modificados a memoria
    movdqu [RSI] , XMM7 ; Pongo 0 en las posiciones de P

    sub R12, 2 ; Decremento el contador de elementos de la fila
    cmp R12, 0 ; Si termino de completar los elementos de la fila
    jne .continuarFila
    add RBX, 4 ; Lo mando a la siguiente fila
    add R15, 4 ; Lo mando a la siguiente fila
    add R14, 4 ; Lo mando a la siguiente fila
    add R13, 4 ; Lo mando a la siguiente fila
    dec RCX    ; Resto uno al contador de filas
    jmp .loop

    .continuarFila:
    add RBX, 2 ; Lo mando a los siguientes dos elementos
    add R15, 2 ; Lo mando a los siguientes dos elementos
    add R14, 2 ; Lo mando a los siguientes dos elementos
    add R13, 2 ; Lo mando a los siguientes dos elementos
    jmp .loop

    .fin:
    pop r15
    pop r14
    pop r13
    pop r12
    pop rbx
    pop rbp
ret

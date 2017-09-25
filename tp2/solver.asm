extern free
extern malloc


solver_set_bnd:
    xor r12, r12
    mov r13, 

    .code:
        mov rbx, rdx
        add rbx, 4
        
        mov r12, rbx
        add r12, N + 4 ; el 4 funca aca?, de donde saco N? 
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
        mulss xmm1, 0.5f
        movss [rbx], xmm1
        add rdx, 4*(N+1) ; que onda las operaciones acá? y el N?

        ;[0, N+1]
        mov rbx, rdx
        sub rbx, 4
        movss xmm1, [rbx]
        add rbx, 4*(N+3)
        movss xmm2, [rbx]
        addss xmm1, xmm2
        mulss xmm1, 0.5f
        movss [rdx], xmm1

        ;[N+1, 0]
        mov rdx, rcx
        add rdx, (N+2)*(N+1)*4
        mov rbx, rdx
        add rbx, 4
        movss xmm1, [rbx]
        sub rbx, (4*(N+3))
        movss xmm2, [rbx]
        addss xmm1, xmm2
        mulss xmm1, 0.5f

        ; [N+1, N+1]
        movss [rdx], xmm1
        add rdx, 4*(N+1)
        mov rbx, rdx
        sub rbx, 4
        movss xmm1, [rbx]
        sub rbx, 4*(N+1)
        movss xmm2, [rbx]
        addss xmm1, xmm2
        mulss xmm1, 0.5f
        movss [rbx], xmm1 

        
          

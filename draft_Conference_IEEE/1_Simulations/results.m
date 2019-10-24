function results(solutions{:},past_a,zel(1),zphist,vphist,alogic,g1hist,b1hist,ghist,z1hist,n1hist,blogic,G1logic,S1logic,N1logic )

    A = solutions{1};past_a(1) = A(:,1);
    Z = solutions{2};   zel(1)=Z(:,1);          zphist=[zphist; Z];
    V = solutions{3};   vphist=[vphist; V];
    A1 = solutions{4};  alogic=A1(:,1);
    g1 = solutions{5};  g1hist=[g1hist; g1];
    B1 = solutions{6};  b1hist=[b1hist B1];     blogic=B1(:,1);
    Gg1 = solutions{7}; ghist=[ghist Gg1];      G1logic=Gg1(:,1);
    Z1 = solutions{8};  z1hist=[z1hist Z1];     S1logic=Z1(:,1);
    N1 = solutions{9};  n1hist=[n1hist N1];     N1logic=N1(:,1);
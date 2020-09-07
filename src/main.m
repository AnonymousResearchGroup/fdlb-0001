output = Hw12P2A('OneComp',1E-9)
%output = Hw12P2A('BInel2dFrm_wEPLHM',1E-9)
fid = fopen('out.json','wt');
fprintf(fid, output);
fclose(fid);

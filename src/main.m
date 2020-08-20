output = Hw12P2A('OneComp',1E-9)
fid = fopen('out.json','wt');
fprintf(fid, output);
fclose(fid);

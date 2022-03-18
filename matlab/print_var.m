function print_var(var, dir)
    f=fopen(dir+".txt", 'w');
    fprintf(f, '%s\n', char(var)); 
    fclose(f);
end


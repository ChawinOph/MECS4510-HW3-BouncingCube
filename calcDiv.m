function div = calcDiv(genes)
%this function takes an array of genes (column is individual and row is
%allele) and calculates the sum of the standard deviation of all alleles
    div = sum(std(genes, 0, 2));

end
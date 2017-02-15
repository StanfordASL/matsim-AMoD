function [shuffled_array, permutation_array] = shuffle(array)

permutation_array=randperm(length(array));


shuffled_array = array(permutation_array);
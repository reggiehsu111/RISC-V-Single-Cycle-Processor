def leaf(a,b,c,d):
    f = (a+b) - (c+d)
    return f

if __name__ == '__main__':
    # Modify your test pattern here
    a = 1
    b = 9
    c = 2
    d = 2
        
    with open('leaf_data.txt', 'w') as f_data:
        f_data.write('{:0>8x}\n'.format(a))
        f_data.write('{:0>8x}\n'.format(b))
        f_data.write('{:0>8x}\n'.format(c))
        f_data.write('{:0>8x}\n'.format(d))

    with open('leaf_data_ans.txt', 'w') as f_ans:
        f_ans.write('{:0>8x}\n'.format(a))
        f_ans.write('{:0>8x}\n'.format(b))
        f_ans.write('{:0>8x}\n'.format(c))
        f_ans.write('{:0>8x}\n'.format(d))
        f_ans.write('{:0>8x}\n'.format(leaf(a,b,c,d)))
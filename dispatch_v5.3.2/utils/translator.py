def dict2sql(arg_dict):
    l_str = ' and '
    str_sql = []

    try:
        for i in arg_dict.keys():
            str_sql.append("{0} = '{1}'".format(i, arg_dict[i]))
        str_sql = l_str.join(str_sql)
    except:
        pass

    return str_sql

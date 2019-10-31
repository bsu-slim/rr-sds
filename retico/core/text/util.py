
def check_overlap(s1, s2):
    s1 = s1.split()
    s2 = s2.split()
    #step through to see what has changed on the frontier
    adds = []
    revokes = []
    #print(s1, 's1/s2', s2)
    if len(s1) == 0:
        return s2, revokes #add everything

    for i,v in enumerate(s2):
        if i>= len(s1): break
        if v == s1[i]: continue # if the two match, do nothing
        # otherwise, revoke what's there, add the new one
        revokes.append(s1[i])
        #print('adding1', v)
        adds.append(v)

    if i >= len(s1):
        #print('adding2', s2[i:])
        adds += s2[i:]

    return adds,revokes


def string_diff_text_iu(module, p, t, s, c, f):

    latest_iu = module.latest_iu()
    if latest_iu is None: latest_text = ''
    else: 
        latest_text = latest_iu.get_text()
    adds, revokes = check_overlap(latest_text, t)
    output_iu = module.create_iu(module.latest_input_iu)
    
    print(t)
    print('revokes', revokes, 'adds', adds)
    if latest_iu is not None:
        latest_iu = latest_iu.previous_iu  #sll


    new_edits = []



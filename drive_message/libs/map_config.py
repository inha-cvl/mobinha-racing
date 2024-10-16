
def get_base_lla(map):
    if map == 'songdo-site':
        base_lla = [37.383378,126.656798,7] # Sondo-Site
    elif map=='songdo':
        base_lla = [37.3888319,126.6428739, 7.369]
    elif map == 'KIAPI':
        base_lla = [35.64588122580907,128.40214778762413, 46.746]
    elif map == 'Pangyo':
        base_lla = [37.39991792889962, 127.11264200835348,7]
    elif map == 'Harbor':
        base_lla = [37.42390324724057, 126.60753475932731, 7]
    elif map == 'KIAPI_Racing':
        base_lla = [35.65492524, 128.39351431, 7]
    elif map == 'KIAPI_Racing_Fast':
        base_lla = [35.65492524, 128.39351431, 7]
    elif map == 'KIAPI_City':
        base_lla = [35.64699249, 128.40116601, 7]
    elif map == 'KCity':
        base_lla = [37.2292221592864,126.76912499027308,7]
    elif map == 'Solchan':
        base_lla = [37.36549921,126.64108444,7]
    return base_lla

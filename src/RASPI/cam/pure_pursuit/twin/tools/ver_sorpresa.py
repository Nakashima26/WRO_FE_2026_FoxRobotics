"""Lanza el visor con el perfil y la semilla del banco sorpresa."""

def main():
    import json,sys
    from pure_pursuit.chassis_twin import main as viewer_main
    cfg=json.load(open('pure_pursuit/twin/tools/carrera_bench.json'))
    seed=sys.argv[1] if len(sys.argv)>1 else '9'
    height=sys.argv[2] if len(sys.argv)>2 else '150'
    args=['chassis_twin','--seed',seed,'--preset','ruta_perfil','--park-pos','random',
          '--surprise-height',height,'--auto-start','--speed','1','--render-every','1',
          '--log-dir',f'/tmp/fox-surprise-ui-s{seed}-h{height}.noindex']
    for k,v in cfg.get('fw',{}).items(): args.extend(['--fw',f'{k}={v}'])
    for k,v in cfg.get('pi',{}).items(): args.extend(['--pi',f'{k}={v!r}'])
    sys.argv=args
    viewer_main()


if __name__ == "__main__":
    main()

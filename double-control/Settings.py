from collections.abc import Mapping

class Settings(Mapping):
    """a metaclass to allow settings to be unpacked into dictionaries"""
    def __init__(cls, name, bases, dct):
        bad = ['__doc__', '__name__', '__qualname__', '__module__', '__defaults__', '__code__', '__dict__', '__annotations__', '__kwdefaults__', '__type_params__']
        cls.__name__ = name
        cls.__bases__ = Mapping.__bases__ + (Mapping,) + bases
        cls.dct=dict([(k, v) for k, v in dct.items() if k not in bad])
        for k, v in cls.dct.items():
            setattr(cls, k, v)
    def withopt(cls, *options_dicts, **options_kw):
        for d in options_dicts + (options_kw,):
            for k, v in d.items():
                cls.dct[k] = v
                setattr(cls, k, v)
        return cls
    def __getitem__(cls, _key):
        return getattr(cls, _key if isinstance(_key, str) else _key[0])
    def __iter__(cls):
        return cls.dct.items().__iter__()
    def __len__(cls):
        return len(cls.dct)
    def keys(cls):
        return cls.dct.keys()
    def __repr__(cls):
        args = ', '.join([f'{k}={v}' for k, v in cls])
        return f'{cls.__name__}({args})'
    def __str__(cls): return repr(cls)

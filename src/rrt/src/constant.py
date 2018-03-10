#/usr/bin/env python
import sys

class _Const(object):
    """ Define Constants """
    class ConstError(TypeError): pass
    class ConstCaseError(): pass
    def __setattr__(self, name, value):
        if name in self.__dict__:
            raise self.ConstError("Can't change const value!")
        if not name.isupper():
            raise self.ConstCaseError("const {} is not all letter are capitalized!".format(name))
        self.__dict__[name] = value

sys.modules[__name__] = _Const()
        
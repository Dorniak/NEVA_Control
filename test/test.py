class A(object):
    value = 0


class B:
    def __init__(self,):
        pass

    def setValueB(self, num):
        A.value = num


# noinspection PyBroadException
def main():
    a = A()
    c = A()
    b = B()
    b.setValueB(5)
    print(f'{A.value}')
    b.setValueB(10)
    print(f'{A.value}')
    b.setValueB(20)
    print(f'{A.value}')
    print(a)
    print('adfh')


if __name__ == '__main__':
    main()

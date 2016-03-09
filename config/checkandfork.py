#-*- encoding:utf-8 -*-
from __future__ import print_function
import httplib
import requests

def main():
    user = raw_input("Por favor digite seu username do Github:")
    user = user.strip()

    urlgit = "https://github.com/{0}".format(user)

    print("Checando usuário {0} no endereço {1}".format(user, urlgit))


    r = requests.get(urlgit)

    if r.status_code != 200:
        print("Obtive erro {0}.".format(r))
        print("O github não tem o usuario {0}. Por favor execute novamente com um usuário válido".format(user))
        return
    else:
        print("Usuário {0} encontrado no Github".format(user))

    print("Checando repositorio clone de robotica16 na conta do usuário {0}".format(user))
    urlrobotica = "{0}/robotica16".format(urlgit)

    r = requests.get(urlrobotica)
    if r.status_code != 200:
        print("Obtive erro {0}.".format(r))
        print("O usuário {0} existe, mas não tem um repositorio fork de https://github.com/mirwox/robotica16".format(user))
        print("Por favor acesse a interface web do Github, vá à página https://github.com/mirwox/robotica16 e clique em fork")
        return
    else:
        print("Repositório robotica16 encontrado no usuário Github chamado {0}".format(user))

    print("""
        Iniciando clonagem do repositório.
    """)

    status = os.system("git clone urlrobotica")
    status = os.system("cd robotica16")
    print("Configurando o repositorio do curso como upstream")
    status = os.system("git remote add upstream https://github.com/mirwox/robotica16")
    status = os.system("cd ../..")
    status = os.system("catkin_make")







if __name__ == "__main__":
    main()

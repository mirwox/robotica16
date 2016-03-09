#-*- encoding:utf-8 -*-
from __future__ import print_function
import httplib
import requests
import os


"""
    Este script Python configura o diretório Git de um aluno
    do curso de robótica e em seguida faz o catkin_make
    para construir os pacotes do neato

    Para baixar direto o arquivo sem precisar clonar o repositório, acesse:
    https://raw.githubusercontent.com/mirwox/robotica16/master/config/checkandfork.py

"""


def main():
    user = raw_input("Por favor digite seu username do Github: ")
    user = user.strip()

    email = raw_input("Por favor digite o e-mail de sua conta do Github ")
    email = email.strip()

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

    status = os.system("git clone {0}".format(urlrobotica))
    os.chdir("robotica16")
    print(status)
    print("Configurando o repositorio do curso como upstream")
    status = os.system("git remote add upstream https://github.com/mirwox/robotica16")
    status = os.system('git config --global user.email "{0}"'.format(email))
    os.mkdir("install_test")
    os.system("git add install_test")
    os.system("git commit -m 'criacao de dir teste'")
    os.system("git push")


    os.chdir("../..")



    print("Para atualizar com o repositório do curso, lembre-se de fazer:\n git fetch upstream")
    status = os.system("catkin_make")







if __name__ == "__main__":
    main()

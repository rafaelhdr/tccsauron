# Introdução #

O Subversion (SVN) é a ferramenta de controle de versão do projeto. Esta página ensina como configurá-lo.

# Política de uso do SVN #
  * **Nunca, _jamais_ dê commit em código que não compila**. Isso só vai deixar seus colegas muito bravos.
  * **Respeite as regras de _ignore_ listadas abaixo**. Elas fazem com que arquivos indesejados, como DLLs, executáveis, .pbd, .ncb etc. não sejam enviadas ao servidor.

# Instruções de instalação #
  * Baixe o [TortoiseSVN](http://tortoisesvn.tigris.org/) e instale-o. Na verdade, qualquer ferramenta que dê acesso ao repositório pode ser usada. As instruções dessa wiki assumem que o usuário está usando o TortoiseSVN
  * Crie um novo repositório (botão direito no Windows Explorer, TortoiseSVN depois "Checkout"). Coloque `https://tccsauron.googlecode.com/svn/trunk/` na URL. Você precisará informar seu nome de usuário do google e a senha gerada [nesta página](http://code.google.com/hosting/settings).
  * Os arquivos começarão a ser baixados. Isso levará bastante tempo.
  * Por fim, configure a lista de "arquivos ignorados" do SVN. Clique com o botão direito no Windows Explorer, e escolha TortoiseSVN -> Settings. Em "Global ignore pattern", entre a seguinte linha ([fonte](http://stackoverflow.com/questions/85353/best-general-svn-ignore-pattern/85377#85377)):

`*.o *.lo *.la #*# .*.rej *.rej .*~ *~ .#* .DS_Store thumbs.db Thumbs.db *.bak *.class *.exe *.dll *.mine *.obj *.ncb *.lib *.log *.idb *.pdb *.ilk *.msi* .res *.pch *.suo *.exp *.*~ *.~* ~*.* cvs CVS .CVS .cvs release Release debug Debug ignore Ignore bin Bin obj Obj *.csproj.user *.user`
> Lembre-se de que isso tudo deve estar em _uma linha_.
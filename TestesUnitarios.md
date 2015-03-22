# Introdução #

Testes unitários são uma maneira poderosa para manter o código confiável e livre de regressões. O Visual Studio 2005/2008 vem com uma ferramenta que ajuda o programador a desenvolvê-los, mas ela é voltada para .NET e nosso projeto é C++ nativo. Felizmente, há um modo de utilizar essa ferramenta da IDE para rodar testes não gerenciados.


# Detalhes #
  1. Nas configurações do projeto que você quer testar, altere "Common Language Runtime Support" para /clr. `*`
  1. Será necessário criar um novo projeto para os testes unitários. Na janela de seleção do Visual Studio, você deve escolher um projeto "Test".
  1. Quando ele for criado, altere, na seção "General", a propriedade "Common Language Runtime Support" de /clr:safe para /clr. Lembre-se de mudar tanto para Debug quanto para Release.
  1. Clicando com o botão direito no ícone do projeto de testes, escolha "Project Dependencies" e marque o seu projeto original como uma dependência deste.
  1. Altere o caminho dos includes para apontar à pasta onde estão os fontes que você deseja testar (e.g. "..\Sonar").
  1. Agora você já pode escrever os testes de maneira similar à feita em C#. Por exemplo, o trecho de código a seguir testa uma função de comparação de números em ponto flutuante:

```
[TestMethod]
void IsEqual()
{
	double x = 0.1;
	float y = 0.05F + 0.05F;
	Assert::IsFalse(x == y);
	Assert::IsTrue(sauron::floating_point::isEqual(x, y));

}
```

O atalho "Ctrl+R, T" roda todos os testes no contexto atual e exibe uma janela que indica se os testes foram bem sucedidos.

# Dicas #

Para testar métodos privados, declare a classe de teste como "friend" da classe a ser testada. Por exemplo, suponha que eu queira testar a classe sauron::Sonar usando o teste `SonarTest` dentro do namespace `SonarUnitTests`. No arquivo "Sonar.h" da classe a ser testada, adiciono a _forward declaration_ da classe de testes:

```
namespace SonarUnitTests{
ref class SonarTest; // "ref" porque a classe de testes é C++/CLI
}
```

E declaro essa classe de testes como friend:

```
namespace sauron
{
class Sonar
{
  public:
  friend ref class SonarUnitTests::SonarTest;
```

Note que não é necessário incluir o header da classe de testes.

# Notas #
`*` Isso é necessário para debugar o código quando se roda um teste unitário. Por algum motivo, não se pode debugar código que esteja em um .cpp em projetos C++ que não sejam /clr.

% Retirado de "PROLOG Programming for Artificial Intelligence"
% Planejador means-ends simples

% Versão para GNU-PROLOG tem a função not definida no arquivo e
% a select renomeada para selectGoal, por conflitar com uma
% função select preexistente

% Relações utilizadas
posicao(a).
posicao(b).
posicao(robo).

bloco(amarelo).
bloco(verde).

plan(State, Goals, [], State) :-
	satisfied(State, Goals).

plan(State, Goals, Plan, FinalState):-
	conc(PrePlan, [Action | PostPlan], Plan),	% Divide o plano
	selectGoal(State, Goals, Goal),			% escolhe um objetivo
	achieves(Action, Goal),				% acha uma ação que satisfaz o objetivo escolhido
	can( Action, Condition),			%
	plan( State, Condition, PrePlan, MidState1),	% torna a ação possível
	apply( MidState1, Action, MidState2),		% aplica a ação
	plan( MidState2, Goals, PostPlan, FinalState).	% trabalha para satisfazer os outros objetivos

% satisfied(State, Goals). - Se os Goals são verdadeiros em State

satisfied(State, []).

satisfied(State, [Goal | Goals]):-
	member(Goal, State),
	satisfied(State, Goals).

selectGoal(State, Goals, Goal):-
	member(Goal, Goals),
	not(member(Goal, State)).



% achieves(Action, Goal) - Verdadeiro se Goal estiver na lista de adições da ação
achieves( Action, Goal):-
	adds( Action, Goals),
	member( Goal, Goals).

% apply(State, Action, NewState) - Quando a ação Action é executada no estado State o estado NewState é atingido
apply( State, Action, NewState) :-
	deletes(Action, DelList),
	delete_all(State, DelList, State1), !,
	adds(Action, AddList),
	conc(AddList, State1, NewState).

% delete_all(L1, L2, Diff) - Diff é L1 - L2
delete_all([], _, []).

delete_all([X | L1], L2, Diff) :-
	member(X, L2), !,
	delete_all(L1, L2, Diff).

delete_all([X | L1], L2, [X | Diff]) :-
	delete_all(L1, L2, Diff).

nao(P):-
	P, !, fail
	;
	true.

diferentes(Pos1, Pos2):-Pos1\==Pos2.

conc([], L, L).
conc([X|L1],L2, [X|L3]):-
	conc(L1, L2, L3).

%--------------------------------------------
% Começo do trecho dependente do problema
%--------------------------------------------

% - Robotica Computacional
% - Planejador para controle de ações de robô móvel

% Espaço de ações para robô

% Mover-se :  mover(Posicao1, Posicao2)




% Pegar bloco :pegar_bloco(Bloco, Posicao)
can(mover(Pos1, Pos2), [posicaoRobo(Pos1)]):-
	posicao(Pos1),
	posicao(Pos2),
	diferentes(Pos1, Pos2).
can(pegar_bloco(Bl, Pos), [posicao(Bl, Pos), posicaoRobo(Pos)]):-
	bloco(Bl),
	posicao(Pos).
can(depositar_bloco(Bl, Pos), [posicaoRobo(Pos), posicao(Bl, robo)]). :- bloco(Bl),
	posicao(Pos).

adds(mover(Pos1, Pos2),[posicaoRobo(Pos2)]).
adds(pegar_bloco(Bl, Pos), [posicao(Bl, robo)]).	% Após o robô pegar o bloco este passa a estar no robô
adds(depositar_bloco(Bl, Pos), [posicao(Bl, Pos)]).


deletes(pegar_bloco(Bl, Pos), [posicao(Bl, Pos)]).	% O bloco não mais está em sua posição antiga, e sim no robô
deletes(depositar_bloco(Bl, Pos), [posicao(Bl, robo)]).
deletes(mover(Pos1, Pos2), [posicaoRobo(Pos1)]).

% Depositar bloco: depositar_bloco(Bloco, Posicao)




writelist([]).

writelist([X|L]) :-
	write(X), write(';'), nl,
	writelist(L).

listtofile(List, File):-
	tell(File),
	writelist(List),
	told,
	tell(user).

not(P):- P,!,fail
	;
	true.
executa:-q1Write(X).


q1(X):-plan([posicaoRobo(a), posicao(verde,a), posicao(amarelo, b)], [posicaoRobo(a), posicao(verde, b), posicao(amarelo, a)], X, Y).
q1Write(X):- q1(X),!, listtofile(X, 'q1.txt').

#include "CustomPlayer.h"
#include <limits.h>
CustomPlayer::CustomPlayer(int index, QThreadPool* threadPool) : Processing(index, threadPool) {
}

void CustomPlayer::buildParameters(Parameters::Handler& parameters) {
  using namespace Parameters;
  // parameters["Openning"] = Text<int>(args.openning); //Parametro da abertura da Hipérbole
  parameters["Distance"] =
      Text<int>(args.dist); // Parametro da distancia antes da bola em que a elipse conclui
  parameters["x"] = Text<int>(args.x); // Parametro da abertura x da elipse
  parameters["y"] = Text<int>(args.y); // Parametro da abertura y da elipse
}

void CustomPlayer::connectModules(const Modules* modules) {
  connect(modules->vision(),
          &Vision::sendFrame,
          this,
          &CustomPlayer::receiveFrame,
          Qt::DirectConnection);

  connect(modules->vision(),
          &Vision::sendField,
          this,
          &CustomPlayer::receiveField,
          Qt::DirectConnection);
}

void CustomPlayer::init(const Modules* modules) {
}

void CustomPlayer::update() {
  shared->field.extract_to(field);
  if (auto f = shared->frame.get_optional_and_reset()) {
    if (auto it = f->allies().findById(index()); it != f->allies().end()) {
      robot = *it;
    }
    frame.emplace(*f);
  }
}

int estado_goleiro = 1; // 1 = Defendendo; 2 = Parado; 3 = Girando para defender ; 4 = Ajustando
                        // angulo; 5 = Se reposiciona;
bool goleiro_flag =
    0; // Flag para angulo do goleiro - 1 = Goleiro Angulo Certo, 0 = Goleiro Angulo Errado

// bool flagLeavingArea = false; Flag para informar se o atacante está saindo da área (No caso em
// que ele não possa entrar na área)
int estado_atacante = 1; // 1 = Aproxima; 2 = Calcula função para pegar a bola; 3 = Percorre a
                         // função; 4 - Ajusta para ir até a bola; 5 - Tenta marcar; 6 - Destreva

Point bolaPos = {}; // Point que recebe a posicao da bola

Point playerPos = {};    // Point que recebe a posicao do atacante
short int estagnado = 0; // int que serve de aviso para quando o atacante estiver estagnado

#define valor 5 // Distancia do goleiro para a linha do gol

// FUNCAO QUE PARAMETRIZA A HIPERBOLE POR CIMA
// std::vector<Point> CustomPlayer::calculaPathTop(Extends<QPointF> bola) {
//   std::vector<Point> temp{};
//   double i = 0;
//   double bx = bola.x() - args.dist;
//   double by = bola.y();
//   double jy = by + args.openning;

//   for (i = jy; i > by; i -= 1) {
//     Point holder((i * i - i * (2 * by + args.openning) + (bx + by * by - by * (-args.openning))),
//                  i);
//     temp.emplace_back(holder);
//   }
//   return temp;
// }

// FUNCAO QUE PARAMETRIZA A HIPERBOLE POR BAIXO
// std::vector<Point> CustomPlayer::calculaPathBot(Extends<QPointF> bola) {
//   std::vector<Point> temp{};
//   double i = 0;
//   double bx = bola.x() - args.dist;
//   double by = bola.y();
//   double jy = by - args.openning;

//   for (i = jy; i < by; i += 1) {
//     Point holder((i * i - i * (2 * by - args.openning) + (bx + by * by - by * args.openning)),
//     i); temp.emplace_back(holder);
//   }
//   return temp;
// }

// FUNCAO QUE PARAMETRIZA A ELIPSE POR BAIXO
std::vector<Point> CustomPlayer::calculaPathBot(Extends<QPointF> bola) {
  std::vector<Point> temp{};
  double i = 0;
  double bx = bola.x() - args.dist;
  double by = bola.y();
  double jy = by - args.y;
  for (i = (3 * PI / 2); i > (PI / 2); i -= (PI / 10)) {
    Point holder((bx + args.x * cos(i)), (jy + args.y * sin(i)));
    temp.emplace_back(holder);
  }
  return temp;
}

// FUNCAO QUE PARAMETRIZA A ELIPSE POR CIMA
std::vector<Point> CustomPlayer::calculaPathTop(Extends<QPointF> bola) {
  std::vector<Point> temp{};
  double i = 0;
  double bx = bola.x() - args.dist;
  double by = bola.y();
  double jy = by + args.y;
  for (i = (3 * PI / 2); i > (PI / 2); i -= (PI / 10)) {
    Point holder((bx + args.x * cos(i)), (jy + (-args.y) * sin(i)));
    temp.emplace_back(holder);
  }
  return temp;
}

void CustomPlayer::exec() {
  if (!field || !frame || !robot) {
    return;
  }

  // // TODO: here...
  // emit sendCommand(...);

  VSSMotion::Spin giraAdjust(2, false); // spin ajuste - goleiro
  VSSRobotCommand rodarAdjA(giraAdjust);

  VSSMotion::Spin giraAdjustB(2, true); // spin ajuste - goleiro
  VSSRobotCommand rodarAdjH(giraAdjustB);

  VSSMotion::Spin giraH(true); // spin horario - goleiro
  VSSRobotCommand rodarH(giraH);

  VSSMotion::Spin giraA(false); // spin anti horario - goleiro
  VSSRobotCommand rodarA(giraA);

  VSSMotion::Stop para; // stop
  VSSRobotCommand parar(para);

  // vai até a bola - atacante
  VSSMotion::GoToPoint GoToBall(frame->ball().position());
  VSSRobotCommand GTB(GoToBall);

  // GOL - atacante
  VSSMotion::GoToPoint TryToScore(field->enemyGoalInsideCenter());
  VSSRobotCommand TS(TryToScore);

  // CENTRO - atacante
  VSSMotion::GoToPoint GoToCenter(field->center());
  VSSRobotCommand GTC(GoToCenter);

  // ponto superior - goleiro
  Extends<QPointF> TopGoal0(robot->x(), field->allyGoalOutsideTop().y());

  VSSMotion::GoToPoint TopGol0(TopGoal0);
  VSSRobotCommand TG0(TopGol0);

  // ponto inferior - goleiro
  Extends<QPointF> BotGoal0(robot->x(), field->allyGoalOutsideBottom().y());

  VSSMotion::GoToPoint BotGol0(BotGoal0);
  VSSRobotCommand BG0(BotGol0);

  // ajuste x - goleiro
  Extends<QPointF> goleiroP(field->enemyGoalOutsideCenter().x() - valor, robot->y());

  // VSSMotion::GoToPoint gPos(field->enemyGoalOutsideCenter());
  VSSMotion::GoToPoint gPos(goleiroP);
  VSSRobotCommand goleiroPos(gPos);

  // altura bola - goleiro
  Extends<QPointF> ballHigh0(robot->x(), frame->ball().y());

  VSSMotion::GoToPoint ballAltura0(ballHigh0);
  VSSRobotCommand BH0(ballAltura0);

  if (robot->id() == 0) { // Goleiro
    switch (estado_goleiro) {
      case 1: // Defendendo
        if (abs(robot->x() - field->enemyGoalOutsideCenter().x() + valor) >
            3) { // Se estiver fora de posicao, va para o estado de se posicionar corretamente
          estado_goleiro = 5;

          // Se estiver alinhado com a bola, va para o estado de parar
        } else if ((frame->ball().y() >= field->allyGoalOutsideTop().y() &&
                    robot->position().y() >= field->allyGoalOutsideTop().y() - 5 &&
                    robot->position().y() <= field->allyGoalOutsideTop().y() + 5) ||
                   (frame->ball().y() <= field->allyGoalOutsideBottom().y() &&
                    robot->position().y() >= field->allyGoalOutsideBottom().y() - 5 &&
                    robot->position().y() <= field->allyGoalOutsideBottom().y() + 5) ||
                   (robot->y() >= frame->ball().y() - 5 && robot->y() <= frame->ball().y() + 5)) {
          estado_goleiro = 2;
        }
        break;

      case 2: // 2 = Parado
        // Se estiver desalinhado com a bola, va para o estado de alinhamento
        if (!((frame->ball().y() >= field->allyGoalOutsideTop().y() &&
               robot->position().y() >= field->allyGoalOutsideTop().y() - 5 &&
               robot->position().y() <= field->allyGoalOutsideTop().y() + 5) ||
              (frame->ball().y() <= field->allyGoalOutsideBottom().y() &&
               robot->position().y() >= field->allyGoalOutsideBottom().y() - 5 &&
               robot->position().y() <= field->allyGoalOutsideBottom().y() + 5) ||
              (robot->y() >= frame->ball().y() - 5 && robot->y() <= frame->ball().y() + 5))) {
          estado_goleiro = 1;
        } else if (robot->distTo(frame->ball()) <
                   15) { // Se estiver proximo o suficiente da bola, va para o estado de girar
          estado_goleiro = 3;
        }
        break;

      case 3: // 3 = Girando para defender
        if (robot->distTo(frame->ball()) >=
            15) { // Se ficar distante o suficiente da bola, flag de angulo fica falsa e vai para o
                  // estado de ajuste de angulo
          estado_goleiro = 4;
          goleiro_flag = 0;
        }
        break;

      case 4:               // 4 = Ajustando angulo
        if (goleiro_flag) { // Se a flag de angulo esta falsa, permanece ajustando

          // Se a flag de angulo estiver verdadeira, e o goleiro nao esta alinhado com a bola, va
          // para o estado de alinahmento
          if (!((frame->ball().y() >= field->allyGoalOutsideTop().y() &&
                 robot->position().y() >= field->allyGoalOutsideTop().y() - 5 &&
                 robot->position().y() <= field->allyGoalOutsideTop().y() + 5) ||
                (frame->ball().y() <= field->allyGoalOutsideBottom().y() &&
                 robot->position().y() >= field->allyGoalOutsideBottom().y() - 5 &&
                 robot->position().y() <= field->allyGoalOutsideBottom().y() + 5) ||
                (robot->y() >= frame->ball().y() - 5 && robot->y() <= frame->ball().y() + 5))) {
            estado_goleiro = 1;
          } else if (robot->distTo(frame->ball()) < 15) { // Se ja estiver alinhado, caso a bola se
                                                          // aproxime, va para o estado de girar
            estado_goleiro = 3;
          }
        }
        break;

      case 5:
        if (abs(robot->x() - field->enemyGoalOutsideCenter().x() + valor) <
            3) { // Quando o goleiro estiver na posicao correta, flag de angulo se torna falso e vai
                 // para estado de ajuste de angulo
          estado_goleiro = 4;
          goleiro_flag = 0;
        }
        break;
    }
    // qDebug() << "case: " << estado_goleiro
    switch (estado_goleiro) {
      case 1:
        if (frame->ball().y() >=
            field->allyGoalOutsideTop().y()) { // Se a bola estiver acima do limite superior da area
                                               // do goleiro, manda o goleiro ficar nesse limite
          emit sendCommand(vssNavigation.run(*robot, TG0));
        } else if (frame->ball().y() <=
                   field->allyGoalOutsideBottom()
                       .y()) { // Se a bola estiver acima do limite inferior da area do goleiro,
                               // manda o goleiro ficar nesse limite
          emit sendCommand(vssNavigation.run(*robot, BG0));
        } else { // Manda o goleiro acaompanhar paralelamente a altura da bola
          emit sendCommand(vssNavigation.run(*robot, BH0));
        }
        break;

      case 2: emit sendCommand(vssNavigation.run(*robot, parar)); break; // Comando de parar

      case 3:
        if (frame->ball().y() >
            robot->y()) { // Se  a bola estiver na parte de cima do campo, rodar no sentido horario
          emit sendCommand(vssNavigation.run(*robot, rodarH));
        } else { // Se  a bola estiver na parte de baixo do campo, rodar no sentido anti horario
          emit sendCommand(vssNavigation.run(*robot, rodarA));
        }
        break;

      case 4:

        // Se a frente do robo estiver entre 0 e pi/2 ou -pi e -pi/2, ajusta no sentido horario (eh
        // o sentido onde o ponto de ajuste esta mais proximo)
        if ((robot->angle() < 1.57 && robot->angle() >= 0) ||
            ((robot->angle() < -1.57) && robot->angle() > -3.14)) {
          emit sendCommand(vssNavigation.run(*robot, rodarAdjH));
          // Se ajustar o agnulo, flag de ajuste fica verdadeira e manda parar
          if ((robot->angle() <= 1.58 && robot->angle() >= 1.56) ||
              (robot->angle() >= -1.58 && robot->angle() <= -1.56)) {
            emit sendCommand(vssNavigation.run(*robot, parar));
            goleiro_flag = 1;
          }
        } else { // Se a frente do robo estiver entre pi/2 e pi ou -pi/2 e 0, ajusta no sentido anti
                 // horario (eh o sentido onde o ponto de ajuste esta mais proximo)
          emit sendCommand(vssNavigation.run(*robot, rodarAdjA));
          // Se ajustar o agnulo, flag de ajuste fica verdadeira e manda parar
          if ((robot->angle() <= 1.58 && robot->angle() >= 1.56) ||
              (robot->angle() >= -1.58 && robot->angle() <= -1.56)) {
            emit sendCommand(vssNavigation.run(*robot, parar));
            goleiro_flag = 1;
          }
        }
        break;

      case 5:
        emit sendCommand(vssNavigation.run(*robot, goleiroPos));
        break; // Manda para o x normal do goleiro
    }
  }

  else if (robot->id() == 1) { // Atacante

    switch (estado_atacante) {
      case 1: // se aproxima da bola

        // Se o atacante estiver nessa posicao por mais de 8 iteracoes, considera que ele esta
        // estagnado e muda para o estado de destravamento
        if (playerPos.y() >= robot->position().y() - 0.1 &&
            playerPos.y() <= robot->position().y() + 0.1 &&
            playerPos.x() <= robot->position().x() + 0.1 &&
            playerPos.x() >= robot->position().x() - 0.1) {
          estagnado++;
        } else {
          estagnado = 0;
          playerPos = robot->position();
        }

        if (estagnado == 8) {
          estado_atacante = 6;
        } else if (robot->distTo(frame->ball().position()) <=
                   50) { // Se a distancia para a bola for <50 vai para o estado que calcula a
                         // funcao
          estado_atacante = 2;
        }
        break;

      case 2: //  caclula a funcao
        if (robot->distTo(frame->ball().position()) >
            50) { // se a distancia do robo para a bola for maior que 50, muda para o estado de
                  // aproximacao
          estado_atacante = 1;

          // Se a bola estiver alinhada com o atacante, vai para o estado de aprioximacao
        } else if (((robot->x() <= frame->ball().x()) && (robot->y() <= frame->ball().y() + 5) &&
                    (robot->y() >= frame->ball().y() - 5))) {
          estado_atacante = 4;
        } else // vai para o estado de execucao de percurso
          estado_atacante = 3;
        break;

      case 3: // faz o percurso
        // Se o atacante estiver nessa posicao por mais de 8 iteracoes, considera que ele esta
        // estagnado e muda para o estado de destravamento
        if (playerPos.y() >= robot->position().y() - 0.5 &&
            playerPos.y() <= robot->position().y() + 0.5 &&
            playerPos.x() <= robot->position().x() + 0.5 &&
            playerPos.x() >= robot->position().x() - 0.5) {
          estagnado++;
        } else {
          estagnado = 0;
          playerPos = robot->position();
        }

        if (estagnado == 8) {
          estado_atacante = 6;
        }

        // Se o atacante estiver na posicao adequada, tenta fazer o gol
        else if (robot->distTo(frame->ball().position()) <= 8.5 &&
                 (robot->x() <= frame->ball().x()) && (robot->y() <= frame->ball().y() + 5) &&
                 (robot->y() >= frame->ball().y() - 5)) {
          estado_atacante = 5;
        }
        // Se a bola tiver mudado de lugar desde o ultimo calculo de percurso, muda para o estado
        // que calcula o percurso
        else if (bolaPos.y() != frame->ball().y() || bolaPos.x() != frame->ball().x()) {
          bolaPos = frame->ball().position();
          estado_atacante = 2;
        }
        // Se o atacante tiver concluido o percurso ou estiver proximo da bola, muda para o estado
        // de ajuste para pegar a bola
        else if (path.empty() ||
                 ((robot->x() <= frame->ball().x()) && (robot->y() <= frame->ball().y() + 5) &&
                  (robot->y() >= frame->ball().y() - 5))) {
          estado_atacante = 4;
        }
        break;

      case 4: // ajusta para pegar a bola
        // Se o atacante estiver nessa posicao por mais de 8 iteracoes, considera que ele esta
        // estagnado e muda para o estado de destravamento
        if (playerPos.y() >= robot->position().y() - 0.5 &&
            playerPos.y() <= robot->position().y() + 0.5 &&
            playerPos.x() <= robot->position().x() + 0.5 &&
            playerPos.x() >= robot->position().x() - 0.5) {
          estagnado++;
        } else {
          estagnado = 0;
          playerPos = robot->position();
        }

        if (estagnado == 4) {
          estado_atacante = 6;
        }
        // Se a distancia para a bola for > 50, muda para o estado de aproximacao
        else if (robot->distTo(frame->ball().position()) > 50) {
          estado_atacante = 1;
        }
        // Se estiver na posicao adquada, tenta fazer o gol
        else if (robot->distTo(frame->ball().position()) <= 8.5 &&
                 (robot->x() <= frame->ball().x()) && (robot->y() <= frame->ball().y() + 5) &&
                 (robot->y() >= frame->ball().y() - 5)) {
          estado_atacante = 5;
        }
        break;

      case 5: // tenta fazer o gol
        // Se o atacante perder a posse da bola, muda para o estado de calcular percurso
        if (!(robot->distTo(frame->ball().position()) <= 8.5 && (robot->x() <= frame->ball().x()) &&
              (robot->y() <= frame->ball().y() + 5) && (robot->y() >= frame->ball().y() - 5))) {
          estado_atacante = 2;
        }
        break;

      case 6: // destrava
        // qDebug() << "travei";
        if (robot->distTo(field->center()) <
            70) { // Destrava ate ficar em uma distancia <70 do meio de campo
          estado_atacante = 1;
          estagnado = 0;
        }
        break;
    }
    // qDebug() << estado_atacante;
    switch (estado_atacante) {
      case 1: emit sendCommand(vssNavigation.run(*robot, GTB)); break; // Vai diretamente ate a bola

      case 2: {
        if (frame->ball().y() >
            robot->position()
                .y()) { // Se a bola estiver mais alta que a do atacante, faz a funcao por baixo
          path = calculaPathBot(frame->ball().position());
          auto it = path.begin();
          int closest = -1;
          double val = INT_MAX;
          int i = 0;
          for (it = path.begin(); it != path.end();
               it++) { // Loop que guarda o index do ponto mais proximo do atacante
            i++;
            if (it->distTo(robot->position()) < val) {
              val = it->distTo(robot->position());
              closest = i;
            }
          }
          it = path.begin();
          for (i = 0; i < closest;
               i++) { // Apaga os pontos da funcao que vem atnes do ponto mais proximo
            path.erase(it);
          }
        } else { // Caso a bola esteja mais baixa que o atacante, calcula a funcao por cima
          path = calculaPathTop(frame->ball().position());
          auto it = path.begin();
          int closest = -1;
          double val = INT_MAX;
          int i = 0;
          for (it = path.begin(); it != path.end();
               it++) { // Loop que guarda o index do ponto mais proximo do atacante
            i++;
            if (it->distTo(robot->position()) < val) {
              val = it->distTo(robot->position());
              closest = i;
            }
          }
          it = path.begin();
          for (i = 0; i < closest;
               i++) { // Apaga os pontos da funcao que vem atnes do ponto mais proximo
            path.erase(it);
          }
        }
        break;
      }

      case 3: {
        auto it = path.begin();
        if (!path.empty() &&
            robot->distTo(path[0]) <= 4) { // Quando passar do ponto da funcao, apaga ele
          path.erase(it);
        }
        if (!path.empty()) { // Se o vetor de potnos nao estiver vazio, manda o atacante para o
                             // ponto seguinte
          VSSMotion::GoToPoint goNext(path[0]);
          VSSRobotCommand next(goNext);
          emit sendCommand(vssNavigation.run(*robot, next));
          break;
        }
      }

      case 4:
        emit sendCommand(vssNavigation.run(*robot, GTB));
        break; // Manda ate a bola, ajustando assim algum problema de angulacao

      case 5:
        emit sendCommand(vssNavigation.run(*robot, TS));
        break; // Vai ate o gol (carregando a bola)

      case 6:
        emit sendCommand(vssNavigation.run(*robot, GTC));
        break; // Vai ate o centro para tentar destravar
    }
  }
}

//      Funcao para caso o goleiro nao pudesse entrar na area, mas optei por nao usa-la
//     case 2: // sai da area
//       if ((robot->distTo(field->center()) > 40)) {
//         flagLeavingArea = true;
//         emit sendCommand(vssNavigation.run(*robot, GTC));
//       } else {
//         flagLeavingArea = false;
//       }
//       break;

void CustomPlayer::receiveField(const Field& field) {
  shared->field = field;
}

void CustomPlayer::receiveFrame(const Frame& frame) {
  shared->frame = frame;
  runInParallel();
}

static_block {
  Factory::processing.insert<CustomPlayer>();
};
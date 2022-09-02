#include "CustomPlayer.h"
#include <limits.h>
CustomPlayer::CustomPlayer(int index, QThreadPool* threadPool) : Processing(index, threadPool) {
}

void CustomPlayer::buildParameters(Parameters::Handler& parameters) {
  using namespace Parameters;
  parameters["Openning"] = Text<int>(args.openning);
  parameters["Distance"] = Text<int>(args.dist);
  parameters["chis"] = Text<int>(args.chis);
  parameters["ipslon"] = Text<int>(args.ipslon);
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

int estado_goleiro = 1; // 1 = Defendendo; 2 = Parado; 3 = Girando; 4 = Ajustando angulo
bool goleiro_flag = 0;

bool flagLeavingArea = false;
int estado_atacante = 1; // 1 = Aproxima; 2 = Ajusta; 3 = Tenta Marcar; 4 - Escapa da area;

Point bolaPos = {};

Point playerPos = {};
short int estagnado = 0;

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

std::vector<Point> CustomPlayer::calculaPathBot(Extends<QPointF> bola) {
  std::vector<Point> temp{};
  double i = 0;
  double bx = bola.x() - args.dist;
  double by = bola.y();
  double jy = by - args.ipslon;
  for (i = (3 * PI / 2); i > (PI / 2); i -= (PI / 10)) {
    Point holder((bx + args.chis * cos(i)), (jy + args.ipslon * sin(i)));
    temp.emplace_back(holder);
  }
  return temp;
}

std::vector<Point> CustomPlayer::calculaPathTop(Extends<QPointF> bola) {
  std::vector<Point> temp{};
  double i = 0;
  double bx = bola.x() - args.dist;
  double by = bola.y();
  double jy = by + args.ipslon;
  for (i = (3 * PI / 2); i > (PI / 2); i -= (PI / 10)) {
    Point holder((bx + args.chis * cos(i)), (jy + (-args.ipslon) * sin(i)));
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

  VSSMotion::GoToPoint GoToBall(frame->ball().position());
  VSSRobotCommand GTB(GoToBall);

  // PONTO QUE AJUSYA X PARA ATAQUE - atacante
  Extends<QPointF> ToBallX(frame->ball().x() - 7, frame->ball().y());

  VSSMotion::GoToPoint GoToBallX(ToBallX); // corre atrás da bola em x
  VSSRobotCommand GTBX(GoToBallX);

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

#define valor 5
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
    //+- pi/2
    switch (estado_goleiro) {
      case 1:
        if (abs(robot->x() - field->enemyGoalOutsideCenter().x() + valor) > 3) {
          estado_goleiro = 5;
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

      case 2:
        if (!((frame->ball().y() >= field->allyGoalOutsideTop().y() &&
               robot->position().y() >= field->allyGoalOutsideTop().y() - 5 &&
               robot->position().y() <= field->allyGoalOutsideTop().y() + 5) ||
              (frame->ball().y() <= field->allyGoalOutsideBottom().y() &&
               robot->position().y() >= field->allyGoalOutsideBottom().y() - 5 &&
               robot->position().y() <= field->allyGoalOutsideBottom().y() + 5) ||
              (robot->y() >= frame->ball().y() - 5 && robot->y() <= frame->ball().y() + 5))) {
          estado_goleiro = 1;
        } else if (robot->distTo(frame->ball()) < 15) {
          estado_goleiro = 3;
        }
        break;

      case 3:
        if (robot->distTo(frame->ball()) >= 15) {
          estado_goleiro = 4;
          goleiro_flag = 0;
        }
        break;

      case 4:
        if (goleiro_flag) {
          if (!((frame->ball().y() >= field->allyGoalOutsideTop().y() &&
                 robot->position().y() >= field->allyGoalOutsideTop().y() - 5 &&
                 robot->position().y() <= field->allyGoalOutsideTop().y() + 5) ||
                (frame->ball().y() <= field->allyGoalOutsideBottom().y() &&
                 robot->position().y() >= field->allyGoalOutsideBottom().y() - 5 &&
                 robot->position().y() <= field->allyGoalOutsideBottom().y() + 5) ||
                (robot->y() >= frame->ball().y() - 5 && robot->y() <= frame->ball().y() + 5))) {
            estado_goleiro = 1;
          } else if (robot->distTo(frame->ball()) < 15) {
            estado_goleiro = 3;
          }
        }
        break;

      case 5:
        if (abs(robot->x() - field->enemyGoalOutsideCenter().x() + valor) < 3) { // revisar valor
          estado_goleiro = 4;
          goleiro_flag = 0;
        }
        break;
    }
    // qDebug() << "case: " << estado_goleiro
    switch (estado_goleiro) {
      case 1:
        if (frame->ball().y() >= field->allyGoalOutsideTop().y()) {
          emit sendCommand(vssNavigation.run(*robot, TG0));
        } else if (frame->ball().y() <= field->allyGoalOutsideBottom().y()) {
          emit sendCommand(vssNavigation.run(*robot, BG0));
        } else {
          emit sendCommand(vssNavigation.run(*robot, BH0));
        }
        break;

      case 2: emit sendCommand(vssNavigation.run(*robot, parar)); break;

      case 3:
        if (frame->ball().y() > robot->y()) {
          emit sendCommand(vssNavigation.run(*robot, rodarH));
        } else {
          emit sendCommand(vssNavigation.run(*robot, rodarA));
        }
        break;

      case 4:
        if ((robot->angle() < 1.57 && robot->angle() >= 0) ||
            ((robot->angle() < -1.57) && robot->angle() > -3.14)) {
          emit sendCommand(vssNavigation.run(*robot, rodarAdjH));
          if ((robot->angle() <= 1.58 && robot->angle() >= 1.56) ||
              (robot->angle() >= -1.58 && robot->angle() <= -1.56)) {
            emit sendCommand(vssNavigation.run(*robot, parar));
            goleiro_flag = 1;
          }
        } else {
          emit sendCommand(vssNavigation.run(*robot, rodarAdjA));
          if ((robot->angle() <= 1.58 && robot->angle() >= 1.56) ||
              (robot->angle() >= -1.58 && robot->angle() <= -1.56)) {
            emit sendCommand(vssNavigation.run(*robot, parar));
            goleiro_flag = 1;
          }
        }
        break;

      case 5: emit sendCommand(vssNavigation.run(*robot, goleiroPos)); break;
    }
  }

  else if (robot->id() == 1) { // Atacante

    switch (estado_atacante) {
      case 1: // se aproxima da bola
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
        } else if (robot->distTo(frame->ball().position()) <= 50) {
          estado_atacante = 2;
        }
        break;

      case 2: //  caclula o percurso
        if (robot->distTo(frame->ball().position()) > 50) {
          estado_atacante = 1;
        } else if (((robot->x() <= frame->ball().x()) && (robot->y() <= frame->ball().y() + 5) &&
                    (robot->y() >= frame->ball().y() - 5))) {
          estado_atacante = 4;
        } else
          estado_atacante = 3;
        break;

      case 3: // faz o percurso
        if (playerPos.y() >= robot->position().y() - 0.5 &&
            playerPos.y() <= robot->position().y() + 0.5 &&
            playerPos.x() <= robot->position().x() + 0.5 &&
            playerPos.x() >= robot->position().x() - 0.5) {
          estagnado++;
        } else {
          estagnado = 0;
          playerPos = robot->position();
        } //

        if (estagnado == 8) {
          estado_atacante = 6;
        }

        else if (robot->distTo(frame->ball().position()) <= 8.5 &&
                 (robot->x() <= frame->ball().x()) && (robot->y() <= frame->ball().y() + 5) &&
                 (robot->y() >= frame->ball().y() - 5)) {
          estado_atacante = 5;
        } else if (bolaPos.y() != frame->ball().y() || bolaPos.x() != frame->ball().x()) {
          bolaPos = frame->ball().position();
          estado_atacante = 2;
        } else if (path.empty() ||
                   ((robot->x() <= frame->ball().x()) && (robot->y() <= frame->ball().y() + 5) &&
                    (robot->y() >= frame->ball().y() - 5))) {
          estado_atacante = 4;
        }
        break;

      case 4: // ajusta para pegar a bola
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
        } else if (robot->distTo(frame->ball().position()) > 50) {
          estado_atacante = 1;
        } else if (robot->distTo(frame->ball().position()) <= 8.5 &&
                   (robot->x() <= frame->ball().x()) && (robot->y() <= frame->ball().y() + 5) &&
                   (robot->y() >= frame->ball().y() - 5)) {
          estado_atacante = 5;
        }
        break;

      case 5: // tenta fazer o gol
        if (!(robot->distTo(frame->ball().position()) <= 8.5 && (robot->x() <= frame->ball().x()) &&
              (robot->y() <= frame->ball().y() + 5) && (robot->y() >= frame->ball().y() - 5))) {
          estado_atacante = 2;
        }
        break;

      case 6: // destrava
        // qDebug() << "travei";
        if (robot->distTo(field->center()) < 70) {
          estado_atacante = 1;
          estagnado = 0;
        }
        break;
    }
    // qDebug() << estado_atacante;
    switch (estado_atacante) {
      case 1: emit sendCommand(vssNavigation.run(*robot, GTB)); break;

      case 2: {
        if (frame->ball().y() > robot->position().y()) {
          path = calculaPathBot(frame->ball().position());
          auto it = path.begin();
          int closest = -1;
          double val = INT_MAX;
          int i = 0;
          for (it = path.begin(); it != path.end(); it++) {
            i++;
            if (it->distTo(robot->position()) < val) {
              val = it->distTo(robot->position());
              closest = i;
            }
          }
          it = path.begin();
          for (i = 0; i < closest; i++) {
            path.erase(it);
          }
        } else {
          path = calculaPathTop(frame->ball().position());
          auto it = path.begin();
          int closest = -1;
          double val = INT_MAX;
          int i = 0;
          for (it = path.begin(); it != path.end(); it++) {
            i++;
            if (it->distTo(robot->position()) < val) {
              val = it->distTo(robot->position());
              closest = i;
            }
          }
          it = path.begin();
          for (i = 0; i < closest; i++) {
            path.erase(it);
          }
        }
        break;
      }

      case 3: {
        auto it = path.begin();
        if (!path.empty() && robot->distTo(path[0]) <= 4) {
          path.erase(it);
        }
        if (!path.empty()) {
          VSSMotion::GoToPoint goNext(path[0]);
          VSSRobotCommand next(goNext);
          emit sendCommand(vssNavigation.run(*robot, next));
          break;
        }
      }

      case 4: emit sendCommand(vssNavigation.run(*robot, GTB)); break;

      case 5: emit sendCommand(vssNavigation.run(*robot, TS)); break;

      case 6: emit sendCommand(vssNavigation.run(*robot, GTC)); break;
    }
  }
}

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
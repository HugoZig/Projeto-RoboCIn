#ifndef PROJECT_UNIFICATION_CUSTOMPLAYER_H
#define PROJECT_UNIFICATION_CUSTOMPLAYER_H

#include "Modules/Modules.h"
#include "Modules/Processing/ProcessingUtils/ProcessingUtils.h"

class CustomPlayer : public Processing {
 public:
  CustomPlayer(int index, QThreadPool* threadPool);

 protected:
  void buildParameters(Parameters::Handler& parameters) override;
  void connectModules(const Modules* modules) override;
  void init(const Modules* modules) override;
  void update() override;
  void exec() override;

 private:
  struct Args {
    Parameters::Arg<int> openning = 9;
    Parameters::Arg<int> dist = 2;
  };
  Args args;

  std::vector<Point> path{};

  struct Shared {
    SharedOptional<Frame> frame;
    SharedOptional<Robot> robot;
    SharedOptional<Field> field;
    SharedValue<QSet<Qt::Key>> keys;
  };
  SharedWrapper<Shared, std::mutex> shared;

  std::optional<Field> field;
  std::optional<Frame> frame;
  std::optional<Robot> robot;

  SSLNavigation sslNavigation;
  VSSNavigation vssNavigation;
  std::vector<Point> calculaPathBot(Extends<QPointF> bola);
  std::vector<Point> calculaPathTop(Extends<QPointF> bola);

 private slots:
  void receiveField(const Field& field);
  void receiveFrame(const Frame& frame);
};

#endif // PROJECT_UNIFICATION_CUSTOMPLAYER_H

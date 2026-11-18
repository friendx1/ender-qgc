#include "LauncherProtocolTest.h"

#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>

#include "LauncherProtocol.h"

// The format is defined by ender/launcher/ender_launcher.py (ELCH v1)

void LauncherProtocolTest::_testEncodeStatusRequest()
{
    const QJsonObject request = QJsonDocument::fromJson(LauncherProtocol::encodeRequest("status", 7)).object();

    QCOMPARE(request.value("v").toInt(), 1);
    QCOMPARE(request.value("seq").toInt(), 7);
    QCOMPARE(request.value("cmd").toString(), QString("status"));
    QVERIFY(!request.contains("step"));
}

void LauncherProtocolTest::_testEncodeStepRequestCarriesTheStep()
{
    const QJsonObject request = QJsonDocument::fromJson(LauncherProtocol::encodeRequest("step", 8, 2)).object();

    QCOMPARE(request.value("cmd").toString(), QString("step"));
    QCOMPARE(request.value("step").toInt(), 2);
    // The launcher wants a whole number, not a decimal one
    QVERIFY(request.value("seq").isDouble());
    QCOMPARE(request.value("seq").toDouble(), 8.0);
}

void LauncherProtocolTest::_testDecodeReply()
{
    const QByteArray datagram =
        R"json({"v":1,"seq":7,"profile":"sim","state":"partial","message":"Planer: Zeitueberschreitung","steps":[)json"
        R"json({"name":"Container","state":"ok","detail":"","can_stop":false},)json"
        R"json({"name":"Planer (SUPER)","state":"failed","detail":"kein Topic","can_stop":true}]})json";

    const auto reply = LauncherProtocol::decodeReply(datagram);

    QVERIFY(reply.has_value());
    QCOMPARE(reply->sequence, quint32(7));
    QCOMPARE(reply->profile, QString("sim"));
    QCOMPARE(reply->state, QString("partial"));
    QCOMPARE(reply->message, QString("Planer: Zeitueberschreitung"));
    QCOMPARE(reply->steps.size(), 2);
    QCOMPARE(reply->steps[0].name, QString("Container"));
    QCOMPARE(reply->steps[0].state, QString("ok"));
    QVERIFY(!reply->steps[0].canStop);
    QCOMPARE(reply->steps[1].state, QString("failed"));
    QCOMPARE(reply->steps[1].detail, QString("kein Topic"));
    QVERIFY(reply->steps[1].canStop);
}

void LauncherProtocolTest::_testBadRepliesAreIgnored()
{
    QVERIFY(!LauncherProtocol::decodeReply("").has_value());
    QVERIFY(!LauncherProtocol::decodeReply("no json").has_value());
    QVERIFY(!LauncherProtocol::decodeReply("[1,2]").has_value());
    // Wrong version
    QVERIFY(!LauncherProtocol::decodeReply(R"json({"v":2,"seq":1,"state":"stopped","steps":[]})json").has_value());
    // No sequence
    QVERIFY(!LauncherProtocol::decodeReply(R"json({"v":1,"state":"stopped","steps":[]})json").has_value());
    // An error answer has no state
    QVERIFY(!LauncherProtocol::decodeReply(R"json({"v":1,"seq":3,"error":"unknown step"})json").has_value());
}

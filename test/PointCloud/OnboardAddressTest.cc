#include "OnboardAddressTest.h"

#include <QtCore/QSettings>
#include <QtCore/QTemporaryDir>
#include <QtTest/QSignalSpy>

#include "OnboardAddress.h"

void OnboardAddressTest::_testDefaultStoredAndEnvironment()
{
    // Own settings folder, the test must not touch the real ones
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, dir.path());
    qunsetenv("ENDER_GOAL_HOST");

    {
        OnboardAddress address;
        QCOMPARE(address.host(), QStringLiteral("192.168.144.50"));
        address.setHost(QStringLiteral("192.168.178.20"));
    }

    // Stored for the next start
    OnboardAddress second;
    QCOMPARE(second.host(), QStringLiteral("192.168.178.20"));

    // The environment variable wins at start, but a typed address still counts afterwards
    qputenv("ENDER_GOAL_HOST", "127.0.0.1");
    OnboardAddress third;
    QCOMPARE(third.host(), QStringLiteral("127.0.0.1"));
    third.setHost(QStringLiteral("10.0.0.7"));
    QCOMPARE(third.host(), QStringLiteral("10.0.0.7"));
    qunsetenv("ENDER_GOAL_HOST");
}

void OnboardAddressTest::_testSetHostTrimsAndResets()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, dir.path());
    qunsetenv("ENDER_GOAL_HOST");

    OnboardAddress address;
    QSignalSpy spy(&address, &OnboardAddress::hostChanged);

    address.setHost(QStringLiteral("  192.168.178.20 \n"));
    QCOMPARE(address.host(), QStringLiteral("192.168.178.20"));
    QCOMPARE(spy.count(), 1);

    address.setHost(QStringLiteral("192.168.178.20"));  // same address, no signal
    QCOMPARE(spy.count(), 1);

    // Empty goes back to the default and forgets the stored address
    address.setHost(QStringLiteral("   "));
    QCOMPARE(address.host(), QStringLiteral("192.168.144.50"));
    QCOMPARE(spy.count(), 2);
    OnboardAddress fresh;
    QCOMPARE(fresh.host(), QStringLiteral("192.168.144.50"));
}

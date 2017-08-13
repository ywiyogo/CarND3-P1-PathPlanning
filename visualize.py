import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import click

@click.command()
@click.argument("output")


def main(output):
    print("Output file: ", output)
    df = pd.read_table(output, sep=';')
    print(df.head(n=5))

    fig = plt.figure(figsize=(16, 8))
    ax1 = fig.add_subplot(311)
    fig.subplots_adjust(top=0.85)
    ax1.set_title('Velocity, Acc, Jerk', fontsize=18, fontweight='bold')
    ax1.set_xlabel('time')
    ax1.set_ylabel('val')
    ax1.plot(df['v_ms'])
    ax1.plot(df['a'])
    ax1.plot(df['jerk'])
    ax1.plot(df['mode'])
    ax1.legend(loc='upper right')
    ax1.grid(linestyle=":")

    ax2 = fig.add_subplot(212)
    ax2.set_title('Yaw and d_dot', fontsize=18, fontweight='bold')
    ax2.set_xlabel('time')
    ax2.set_ylabel('val')
    ax2.plot(df['yaw'])
    ax2.plot(df['d'])
    ax2.plot(df['d_yaw'])
    ax2.plot(df['d_dot'])
    ax2.plot(df['d_dotdot'])

    ax2.legend(loc='upper right')
    ax2.grid(linestyle=":")

    #fig.tight_layout()
    plt.savefig('experiments.png')

    fig2 = plt.figure(figsize=(16, 8))
    ax3 = fig2.add_subplot(111)
    ax3.set_title('Track Record', fontsize=18, fontweight='bold')
    ax3.set_xlabel('x')
    ax3.set_ylabel('y')
    ax3.grid(linestyle=":")
    ax3.plot(df['x'],df['y'])

    fig3 = plt.figure(figsize=(16, 8))
    ax4 = fig3.add_subplot(111)
    ax4.set_title('Simulation Update Time', fontsize=18, fontweight='bold')
    df['MovAvg'] = df['dt'].rolling(window=50).mean()
    fittedMA = np.linspace(0, 1, len(df['dt']))
    yinterp = np.interp(fittedMA, df['MovAvg'], range(0,len(df['dt'])))
    ax4.set_xlabel('t')
    ax4.set_ylabel('delay')
    ax4.grid(linestyle=":")
    ax4.plot(df['dt'])
    ax4.plot(df['MovAvg'])
    plt.show()

if __name__ == '__main__':
    main()
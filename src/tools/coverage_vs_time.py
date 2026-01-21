import ast, pandas as pd, numpy as np, matplotlib.pyplot as plt

df = pd.read_csv("results/modes_T95_bayes_8.csv")
T = max(len(ast.literal_eval(h)) for h in df['coverage_history'])

def expand(hist):
    vals = ast.literal_eval(hist)
    return np.pad(vals, (0, T - len(vals)), constant_values=vals[-1] if vals else 0)

df['coverage_series'] = df['coverage_history'].apply(expand).apply(lambda x: np.array(x))

for env in ['clean', 'adversarial']:
    plt.figure()
    for mode in df['mode'].unique():
        subset = df[(df['env']==env) & (df['mode']==mode)]
        if subset.empty: continue
        stack = np.stack(subset['coverage_series'].to_list())
        mean = stack.mean(axis=0)
        sem = stack.std(axis=0)/np.sqrt(len(stack))
        plt.plot(mean, label=f"{mode} (mean)")
        plt.fill_between(range(T), mean-sem, mean+sem, alpha=0.2)
    plt.xlabel("Timestep")
    plt.ylabel("Coverage")
    plt.title(f"Coverage vs Time ({env})")
    plt.legend()
    plt.tight_layout()
    plt.savefig(f"results/coverage_vs_time_{env}.png", dpi=150)
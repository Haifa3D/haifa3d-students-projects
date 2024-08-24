from sklearn.tree import _tree
from sklearn.ensemble import RandomForestClassifier
from sklearn.tree import export_text
import numpy as np
import pickle
from typing import List

def main():
    with open("code/ML/saved_RF_calssifier_2axis.pickle", 'rb') as infile:
        model: RandomForestClassifier = pickle.load(infile)
    outfile = open('forrest.hpp', 'wt', encoding='utf-8')
    print("#ifndef RF_PREDICT\n#define RF_PREDICT\nvoid rf_predict(float *in, int *cls_hist){", file=outfile)
    print(f"#define NUM_TREES {len(model.estimators_)}", file=outfile)
    print(f"#define NUM_CLASSES {model.n_classes_}", file=outfile)
    print(f"#define NUM_FEATURES {model.estimators_[0].tree_.n_features}", file=outfile)
    for idx, t in enumerate(model.estimators_):
        print(f'\t// tree {idx}', file=outfile)
        tree_rules = export_text(t, feature_names=[f'in[{i}]' for i in range(t.tree_.n_features)])
        # print(tree_rules)
        convert_tree_to_c(tree_rules.strip().split('\n'), outfile=outfile)
    print('}\n#endif', file=outfile)

def convert_tree_to_c(rules: List[str] , var_pref='in', outfile=None):
    level_p = 0
    rules.append('|')
    for idx, line in enumerate(rules):
        level = line.count('|')
        if level < level_p:
            for l in reversed(range(level,level_p)):
                print('    ' * l + '}', file=outfile)
        print('    ' * level, end='', file=outfile)
        if 'class' in line:
            class_ = int(float(line.split(':')[-1].strip()))
            print(f'cls_hist[{class_}]++;', file=outfile)
        else:
            if '<=' in line:
                cond = line.split('---')[-1].strip()
                print(f'if({cond})' + ' {', file=outfile)
            elif '>' in line:
                print('else {', file=outfile)
        level_p = level
    print('', file=outfile)


if __name__ == '__main__':
    main()